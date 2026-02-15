# FRC Robot Code Audit Report

## Executive Summary

This document investigates two specific issues reported with the 2026 FRC robot:
1. **Forward direction changes** when updating the drivetrain pose using the Limelight
2. **Control loop timing warnings** occurring approximately once per second

Additionally, general code quality issues and common FRC programming mistakes are documented.

---

## Issue 1: Forward Direction Changes When Using Limelight Pose

### Symptom
When the robot updates its position estimate using the Limelight camera, the "forward" direction on the robot changes unexpectedly, as if the robot was started facing the wrong direction.

### Root Cause Analysis

The issue is in `RobotContainer.java:223-236` in the `updateDrivetrainRobotPerspective()` method:

```java
public void updateDrivetrainRobotPerspective() {
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Robot.LIMELIGHTS[0]);
    if (llMeasurement != null){
        drivetrain.resetPose(llMeasurement.pose);  // <-- THIS IS THE PROBLEM
    }
    Rotation2d forward;
    if (Robot.alliance == Alliance.Red) {
        forward = new Rotation2d(Math.PI);
    } else {
        forward = new Rotation2d(0);
    }
    drivetrain.setOperatorPerspectiveForward(forward);
}
```

**Problem 1: Using MegaTag1 instead of MegaTag2**

The code uses `getBotPoseEstimate_wpiBlue()` which is the **MegaTag1** algorithm. MegaTag1 calculates the robot's rotation purely from visual AprilTag geometry, which can be inaccurate due to:
- Camera calibration errors
- AprilTag ambiguity (tags can appear flipped 180 degrees)
- Perspective distortion at oblique viewing angles

The robot's gyroscope is far more accurate for rotation estimation than visual-only pose calculation.

**Problem 2: `resetPose()` overwrites the gyro heading**

When `drivetrain.resetPose(llMeasurement.pose)` is called, it completely resets the internal odometry state, including the robot's heading. The Limelight's rotation estimate replaces the gyro's accumulated heading, which:
1. Introduces rotation error from the Limelight
2. Causes "forward" to point in the wrong direction from the operator's perspective

**Problem 3: Inconsistency with the continuous pose fusion**

In `Robot.java:95-107`, the periodic pose fusion correctly uses MegaTag2 and ignores rotation:

```java
LimelightHelpers.SetRobotOrientation(LIMELIGHTS[i], botState.Pose.getRotation().getDegrees(), ...);
PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHTS[i]);
// ...
drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
    VecBuilder.fill(.9, .9, 999999));  // 999999 = ignore rotation
```

This correctly:
1. Feeds the gyro heading TO the Limelight for MegaTag2 calculation
2. Uses `addVisionMeasurement` (not `resetPose`) for fusion
3. Sets rotation standard deviation to 999999 (effectively ignoring Limelight rotation)

But the Y button handler uses a completely different approach that contradicts this design.

### Recommended Fix

The `updateDrivetrainRobotPerspective()` method should either:

**Option A: Only reset translation, preserve gyro heading**
```java
public void updateDrivetrainRobotPerspective() {
    // First, send gyro heading to Limelight for MegaTag2
    var botState = drivetrain.getState();
    LimelightHelpers.SetRobotOrientation(Robot.LIMELIGHTS[0],
        botState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    // Use MegaTag2 (requires gyro input)
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Robot.LIMELIGHTS[0]);
    if (llMeasurement != null && llMeasurement.tagCount > 0) {
        // Create new pose with Limelight translation but KEEP current rotation
        Pose2d correctedPose = new Pose2d(
            llMeasurement.pose.getTranslation(),
            botState.Pose.getRotation()  // Keep gyro heading
        );
        drivetrain.resetPose(correctedPose);
    }

    // Set operator perspective (unchanged)
    Rotation2d forward = Robot.alliance == Alliance.Red
        ? new Rotation2d(Math.PI)
        : new Rotation2d(0);
    drivetrain.setOperatorPerspectiveForward(forward);
}
```

**Option B: Use addVisionMeasurement with very high confidence**
```java
public void updateDrivetrainRobotPerspective() {
    var botState = drivetrain.getState();
    LimelightHelpers.SetRobotOrientation(Robot.LIMELIGHTS[0],
        botState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Robot.LIMELIGHTS[0]);
    if (llMeasurement != null && llMeasurement.tagCount > 0) {
        // Very tight standard deviations for X,Y, huge for rotation (ignore it)
        drivetrain.addVisionMeasurement(
            llMeasurement.pose,
            llMeasurement.timestampSeconds,
            VecBuilder.fill(0.01, 0.01, 999999)  // Trust X,Y heavily, ignore rotation
        );
    }

    // ... rest unchanged
}
```

---

## Issue 2: Control Loop Timing Warnings

### Symptom
Approximately once per second, the robot reports that control loop times are too slow (exceeding the 20ms deadline).

### Root Cause Analysis

The periodic loop has several performance bottlenecks:

#### Bottleneck 1: Excessive SmartDashboard Calls Per Cycle

**ShooterSubsystem.periodic()** (called 3 times for John, Jawbreaker, Taylor):
```java
SmartDashboard.putNumber(name + " Shooter Speed Target", shooterSpeed);  // String concat!
SmartDashboard.putBoolean(name + " Is Shooting", isShooting);
shootPIDMotor.putPV();      // 2 more SmartDashboard calls + 2 CAN reads
shootPIDMotor.putCurrent(); // 1 SmartDashboard call + 1 CAN read
SmartDashboard.putNumber(name + " Feeder Speed Target", feederSpeed);
SmartDashboard.putBoolean(name + " Is Feeding", isFeeding);
feedPIDMotor.putPV();       // 2 more SmartDashboard calls + 2 CAN reads
feedPIDMotor.putCurrent();  // 1 SmartDashboard call + 1 CAN read
SmartDashboard.putBoolean(name + " Beambreak", beamBreak.get());
```

**Per shooter per cycle:** ~9 SmartDashboard writes, ~6 CAN reads
**Total for 3 shooters:** ~27 SmartDashboard writes, ~18 CAN reads

**FloorFeedSubsystem.periodic():**
```java
for (int i = 0; i < shooters.length; i++) {
    SmartDashboard.putBoolean("shooter " + i + " needs floor", shooters[i].needsFloorFeed());
}
SmartDashboard.putBoolean("needs floor", needsToRun);
motor.putCurrent();  // SmartDashboard + CAN read
SmartDashboard.putBoolean("Floor Feed is feeding", isFeeding);
SmartDashboard.putNumber("Floor Feed motor speed centre", motorSpeedCentre);
SmartDashboard.putNumber("Floor Feed motor real speed", motor.getVelocity());  // CAN read
```

**IntakeSubsystem.periodic():**
```java
SmartDashboard.putNumber("Intake Speed", intakeMotorSpeed);
SmartDashboard.putBoolean("Is Intaking", isIntaking);
SmartDashboard.putNumber("Intake Actual Speed", intakePIDMotor.getVelocity());  // CAN read
SmartDashboard.putNumber("Tilt Position Target", tiltPositionAbsolute);
SmartDashboard.putNumber("Tilt Actual Position", tiltPIDMotor.getPosition());   // CAN read
```

**Robot.robotPeriodic():**
```java
SmartDashboard.putBoolean("Is Manual Mode", m_robotContainer.isManualMode);
// Limelight network calls...
SmartDashboard.putData("Bot Field", m_botField);
SmartDashboard.putData("Limelight Field", m_limelightField);
```

**Telemetry.telemeterize():**
```java
for (int i = 0; i < 4; ++i) {
    SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);  // 4x per cycle
}
```

**Total SmartDashboard calls per cycle: ~50+ writes**

#### Bottleneck 2: Blocking CAN Bus Reads

`PIDMotor.getVelocity()`, `getPosition()`, and `getCurrent()` are **synchronous blocking calls**:

```java
public double getVelocity() {
    StatusSignal<AngularVelocity> status = motor.getVelocity();  // BLOCKS for CAN response
    StatusCode code = status.getStatus();
    // error handling...
    return status.getValueAsDouble();
}
```

Each call waits for a CAN frame response. With 8 motors being queried multiple times per cycle, this adds up.

**CAN reads per cycle:**
- 3 shooters x (2 velocity + 2 position + 2 current) = 18 reads
- 1 floor feed x (1 velocity + 1 current) = 2 reads
- 1 intake x (2 velocity + 2 position) = 4 reads
- **Total: ~24 blocking CAN reads per cycle**

#### Bottleneck 3: String Concatenation in Hot Path

Multiple subsystems concatenate strings every cycle:
```java
SmartDashboard.putNumber(name + " Shooter Speed Target", shooterSpeed);
```

String concatenation allocates new objects, increasing GC pressure.

#### Bottleneck 4: NetworkTables Flush

In `Robot.robotPeriodic()`, the Limelight code calls `SetRobotOrientation` which internally calls:
```java
if(flush) {
    Flush();  // NetworkTableInstance.getDefault().flush()
}
```

`Flush()` is a synchronous network I/O operation that blocks until all pending NetworkTables updates are sent to all connected clients.

### Why ~Once Per Second?

The timing warnings appear periodically because:
1. **Garbage collection** - The JVM periodically runs GC due to string allocations
2. **NetworkTables sync** - NT4 has internal timing that can cause periodic delays
3. **CAN bus contention** - The swerve drive odometry loop runs at 250Hz, competing with telemetry reads

### Recommended Fixes

**Fix 1: Reduce SmartDashboard frequency**

Only update telemetry every N cycles:
```java
private int telemetryCounter = 0;
private static final int TELEMETRY_PERIOD = 5;  // Every 100ms instead of 20ms

@Override
public void periodic() {
    // Critical control logic runs every cycle
    if (isShooting) {
        shootPIDMotor.setVelocityTarget(shooterSpeed);
    } else {
        shootPIDMotor.setPercentOutput(0);
    }

    // Telemetry only every 5th cycle
    if (++telemetryCounter >= TELEMETRY_PERIOD) {
        telemetryCounter = 0;
        SmartDashboard.putNumber(name + " Shooter Speed Target", shooterSpeed);
        // ... other telemetry
    }
}
```

**Fix 2: Cache SmartDashboard keys**

Pre-compute string keys in constructor:
```java
private final String shooterSpeedKey;
private final String isShootingKey;
// ...

public ShooterSubsystem(String name, ...) {
    this.shooterSpeedKey = name + " Shooter Speed Target";
    this.isShootingKey = name + " Is Shooting";
    // ...
}
```

**Fix 3: Use asynchronous CAN reads**

Use Phoenix 6's signal refresh and latency compensation:
```java
// In constructor - set up refresh rate
motor.getVelocity().setUpdateFrequency(50);  // 50 Hz instead of on-demand
motor.getPosition().setUpdateFrequency(50);

// In periodic - read cached values (non-blocking)
public double getVelocity() {
    return motor.getVelocity().getValue();  // Returns cached value, doesn't block
}
```

**Fix 4: Remove Flush() call**

Use `SetRobotOrientation_NoFlush` instead:
```java
LimelightHelpers.SetRobotOrientation_NoFlush(LIMELIGHTS[i],
    botState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
```

---

## Additional Code Issues Found

### Bug: JustShootCmd feeds wrong shooter

In `JustShootCmd.java:46`:
```java
jawbreakerShooterSubsystem.setIsFeeding(false);  // Should be taylorShooterSubsystem
```

Copy-paste error - Taylor's feeder is never turned off.

### Bug: ClimbSubsystem has no periodic() method

`ClimbSubsystem.java` comment states:
```java
// TODO: no periodic. boken
```

The climb system cannot function without a periodic method to drive the motor.

### Issue: IntakeSubsystem tilt control is disabled

```java
public void setTiltPosition(double position) {
    // tiltPositionAbsolute = position;
    // tiltPIDMotor.setTarget(tiltPositionAbsolute, tiltMaxSpeed, tiltMaxAccel);
}
```

Tilt positioning is completely commented out.

### Issue: Alliance color not updated during match

`Robot.updateAlliance()` is only called in the constructor. If the DriverStation alliance changes (or is initially unknown), the robot won't adapt. Consider calling this periodically or in `autonomousInit()`/`teleopInit()`.

### Issue: Duplicate command binding

In `RobotContainer.java`:
```java
// Line 158
m_driverController.leftBumper().whileTrue(new DetectFuelCmd(drivetrain));

// Line 218
m_driverController.leftBumper().whileTrue(new DetectFuelCmd(drivetrain));
```

The same binding is registered twice, creating two separate command instances.

### Issue: PIDMotor init() sleeps on main thread

```java
private void init() {
    if (!initialized) {
        sleep();      // 20ms
        resetAll();
        sleep();      // 20ms
        sleep();      // 20ms
        updatePIDF();
        sleep();      // 20ms
        // ...
    }
}
```

With 8 motors, that's 8 x 80ms = 640ms of blocking sleeps during robot startup.

### Issue: System.currentTimeMillis() for timing

In `FloorFeedSubsystem.java:114`:
```java
motor.setVelocityTarget(-getVelocityAtTime(System.currentTimeMillis() / 1_000.0d));
```

Should use `Timer.getFPGATimestamp()` for consistent timing with the rest of the robot.

### Issue: Using wildcard CAN bus

In `PIDMotor.java:75`:
```java
motor = new TalonFX(deviceID, "*");
```

Using `"*"` as the CAN bus name means "search all buses". This works but is slower than specifying the exact bus name.

---

## Performance Summary Table

| Location | Issue | Impact | Priority |
|----------|-------|--------|----------|
| All subsystems | Excessive SmartDashboard calls | High | HIGH |
| PIDMotor | Blocking CAN reads | Medium-High | HIGH |
| LimelightHelpers | NetworkTables flush | Medium | MEDIUM |
| All subsystems | String concatenation | Low-Medium | LOW |
| updateDrivetrainRobotPerspective | Wrong Limelight method | High (causes issue #1) | CRITICAL |
| updateDrivetrainRobotPerspective | resetPose() overwrites gyro | High (causes issue #1) | CRITICAL |

---

## Recommended Action Plan

### Immediate (Before Next Match)
1. Fix `updateDrivetrainRobotPerspective()` to use MegaTag2 and preserve gyro heading
2. Fix JustShootCmd copy-paste bug
3. Reduce SmartDashboard update frequency to every 5th cycle

### Short-term (Before Competition)
1. Configure TalonFX signal update frequencies in constructors
2. Remove duplicate DetectFuelCmd binding
3. Implement ClimbSubsystem.periodic()
4. Cache SmartDashboard string keys

### Long-term (Post-Season)
1. Use `SetRobotOrientation_NoFlush`
2. Specify explicit CAN bus names
3. Replace `System.currentTimeMillis()` with FPGA timestamp
4. Consider using AdvantageKit or similar for structured logging
