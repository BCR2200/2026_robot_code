# Telemetry Performance: Problem Analysis and Best Practices

## The Problem

### Current Implementation

Our robot code uses direct `SmartDashboard.putNumber()`/`putBoolean()` calls scattered throughout subsystem `periodic()` methods. This approach has several performance problems:

```java
// ShooterSubsystem.periodic() - Called 3x (John, Jawbreaker, Taylor)
@Override
public void periodic() {
    SmartDashboard.putNumber(name + " Shooter Speed Target", shooterSpeed);  // String concat
    SmartDashboard.putBoolean(name + " Is Shooting", isShooting);
    shootPIDMotor.putPV();      // getPosition() + getVelocity() = 2 blocking CAN reads
    shootPIDMotor.putCurrent(); // getCurrent() = 1 blocking CAN read
    SmartDashboard.putNumber(name + " Feeder Speed Target", feederSpeed);
    SmartDashboard.putBoolean(name + " Is Feeding", isFeeding);
    feedPIDMotor.putPV();       // 2 more blocking CAN reads
    feedPIDMotor.putCurrent();  // 1 more blocking CAN read
    SmartDashboard.putBoolean(name + " Beambreak", beamBreak.get());
    // ... motor control logic mixed with telemetry
}
```

### Why This Is Slow

| Issue | Impact | Per Cycle Cost |
|-------|--------|----------------|
| **String concatenation** | Allocates new objects, GC pressure | ~50+ string operations |
| **Blocking CAN reads** | Waits for CAN bus response | ~24 reads @ ~0.5ms each |
| **SmartDashboard overhead** | NT4 serialization + network | ~50 writes |
| **Mixed concerns** | Telemetry runs even when not needed | Always on |
| **NetworkTables flush** | Synchronous network I/O | 1 per Limelight call |

**Result:** Loop overruns (~1/second) as the 20ms deadline is exceeded.

### The Core Insight

> "Data logging increases CPU load on the roboRIO and can lead to loop overruns. The overhead comes from expensive data-reading operations, particularly CAN bus queries—not logging itself."
> — [WPILib Documentation](https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html)

The problem isn't `SmartDashboard.putNumber()` itself—it's the pattern of reading hardware values synchronously every cycle just to display them.

---

## Industry Best Practices

### 1. Separate Data Collection from Control Logic

Control loops should not be responsible for telemetry. This separation:
- Keeps control code fast and focused
- Allows telemetry to run at a different rate
- Makes code more testable

### 2. Use Annotation-Based Logging

Modern FRC logging libraries use Java annotations to automatically log fields without manual `put` calls:

| Library | Approach | Pros | Cons |
|---------|----------|------|------|
| **[Epilogue](https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html)** (WPILib built-in) | `@Logged` annotation | No dependencies, official support | Java only |

### 3. Cache Hardware Reads

Instead of reading CAN values on-demand, configure update frequencies and read cached values:

```java
// Phoenix 6: Configure in constructor
motor.getVelocity().setUpdateFrequency(50);  // 50 Hz background updates
motor.getPosition().setUpdateFrequency(50);

// In periodic: Non-blocking cached read
double velocity = motor.getVelocity().getValue();  // Returns cached value
```

### 4. Pre-compute String Keys

Avoid string concatenation in hot paths:

```java
// BAD: Allocates new string every cycle
SmartDashboard.putNumber(name + " Speed", speed);

// GOOD: Compute once in constructor
private final String speedKey;
public MySubsystem(String name) {
    this.speedKey = name + " Speed";
}
// Then use cached key
SmartDashboard.putNumber(speedKey, speed);
```

---

## Recommended Solution: Cache CAN reads and use Epilogue

Epilogue is WPILib's built-in annotation-based logging system. It's the recommended approach because:
- No third-party dependencies
- Officially supported
- Runs offset from control loops to reduce CPU spikes
- Configurable importance levels

### Setup

**1. Add `@Logged` to your subsystems:**

```java
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;

@Logged
public class ShooterSubsystem extends SubsystemBase {

    // These will be automatically logged
    private double shooterSpeed = 42;
    private boolean isShooting = false;
    private double feederSpeed = 100;
    private boolean isFeeding = false;

    // Exclude internal implementation details
    @NotLogged
    private final Interpolator shooterAngleInterpolator;

    @NotLogged
    public PIDMotor shootPIDMotor;  // Log motor data via custom logger instead

    // ... rest of subsystem
}
```

**2. Create custom loggers for vendor classes:**

```java
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(PIDMotor.class)
public class PIDMotorLogger extends ClassSpecificLogger<PIDMotor> {

    public PIDMotorLogger() {
        super(PIDMotor.class);
    }

    @Override
    public void update(EpilogueBackend backend, PIDMotor motor) {
        // These reads happen during Epilogue's logging phase, not control loop
        backend.log("Position", motor.getPosition());
        backend.log("Velocity", motor.getVelocity());
        backend.log("Current", motor.getCurrent());
    }
}
```

**3. Bind Epilogue in Robot.java:**

```java
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;

@Logged
public class Robot extends TimedRobot {

    public Robot() {
        // Start data logging to USB drive
        DataLogManager.start();

        // Configure Epilogue
        Epilogue.configure(config -> {
            // Only log important things during matches
            config.minimumImportance = Logged.Importance.DEBUG;

            // Custom root path in NetworkTables
            config.root = "Telemetry";
        });

        // Bind logger - runs at robot frequency, offset by half phase
        Epilogue.bind(this);
    }
}
```

**4. Clean up periodic() methods:**

```java
@Logged
public class ShooterSubsystem extends SubsystemBase {

    // Fields are logged automatically by Epilogue
    private double shooterSpeed = 42;
    private boolean isShooting = false;

    @Override
    public void periodic() {
        // ONLY control logic here - no SmartDashboard calls!
        if (isShooting) {
            shootPIDMotor.setVelocityTarget(shooterSpeed);
        } else {
            shootPIDMotor.setPercentOutput(0);
        }

        if (isFeeding) {
            feedPIDMotor.setVelocityTarget(feederSpeed);
        } else if (beamBreak.get()) {
            feedPIDMotor.setPercentOutput(PRELOAD_SPEED);
        } else {
            feedPIDMotor.setPercentOutput(0);
        }
    }
}
```

---

## Example: Refactored ShooterSubsystem

Here's a complete before/after comparison:

### Before (Current Code)

```java
public class ShooterSubsystem extends SubsystemBase {
    private String name;
    private boolean isShooting = false;
    public double shooterSpeed = 42;
    private boolean isFeeding = false;
    public double feederSpeed = 100;
    public PIDMotor shootPIDMotor;
    public PIDMotor feedPIDMotor;
    private DigitalInput beamBreak;
    // ... constructor and other methods ...

    @Override
    public void periodic() {
        // TELEMETRY (slow)
        SmartDashboard.putNumber(name + " Shooter Speed Target", shooterSpeed);
        SmartDashboard.putBoolean(name + " Is Shooting", isShooting);
        shootPIDMotor.putPV();      // Blocking CAN read + SmartDashboard
        shootPIDMotor.putCurrent(); // Blocking CAN read + SmartDashboard
        SmartDashboard.putNumber(name + " Feeder Speed Target", feederSpeed);
        SmartDashboard.putBoolean(name + " Is Feeding", isFeeding);
        feedPIDMotor.putPV();       // Blocking CAN read + SmartDashboard
        feedPIDMotor.putCurrent();  // Blocking CAN read + SmartDashboard
        SmartDashboard.putBoolean(name + " Beambreak", beamBreak.get());

        // CONTROL LOGIC (fast)
        if (isShooting) {
            shootPIDMotor.setVelocityTarget(shooterSpeed);
        } else {
            shootPIDMotor.setPercentOutput(0);
        }

        if (isFeeding) {
            feedPIDMotor.setVelocityTarget(feederSpeed);
        } else if (beamBreak.get()) {
            feedPIDMotor.setPercentOutput(PRELOAD_SPEED);
        } else {
            feedPIDMotor.setPercentOutput(0);
        }
    }
}
```

### After (Epilogue-Based)

```java
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;

@Logged
public class ShooterSubsystem extends SubsystemBase {

    // Automatically logged by Epilogue
    private final String name;
    private boolean isShooting = false;
    private double shooterSpeed = 42;
    private boolean isFeeding = false;
    private double feederSpeed = 100;

    // Custom logger handles these (see PIDMotorLogger)
    @Logged(name = "ShootMotor")
    public PIDMotor shootPIDMotor;

    @Logged(name = "FeedMotor")
    public PIDMotor feedPIDMotor;

    // Logged with custom name
    @Logged(name = "BeamBroken")
    public boolean isBeamBroken() {
        return !beamBreak.get();  // Invert for clarity: true = ball present
    }

    // Internal implementation details - don't log
    @NotLogged
    private DigitalInput beamBreak;

    @NotLogged
    private final Interpolator shooterAngleInterpolator;

    // ... constructor unchanged ...

    @Override
    public void periodic() {
        // ONLY control logic - telemetry handled by Epilogue
        if (isShooting) {
            shootPIDMotor.setVelocityTarget(shooterSpeed);
        } else {
            shootPIDMotor.setPercentOutput(0);
        }

        if (isFeeding) {
            feedPIDMotor.setVelocityTarget(feederSpeed);
        } else if (beamBreak.get()) {
            feedPIDMotor.setPercentOutput(PRELOAD_SPEED);
        } else {
            feedPIDMotor.setPercentOutput(0);
        }
    }
}
```

### PIDMotorLogger (Custom Logger for TalonFX Wrapper)

```java
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(PIDMotor.class)
public class PIDMotorLogger extends ClassSpecificLogger<PIDMotor> {

    public PIDMotorLogger() {
        super(PIDMotor.class);
    }

    @Override
    public void update(EpilogueBackend backend, PIDMotor motor) {
        // Log motor state - reads happen during Epilogue phase
        backend.log("Position", motor.getPosition());
        backend.log("Velocity", motor.getVelocity());
        backend.log("Current", motor.getCurrent());
        backend.log("Target", motor.target);
    }
}
```

### Configure CAN signal update rates

In `PIDMotor.java` constructor:

```java
private void init() {
    // ... existing init code ...

    // Configure signal update frequencies (non-blocking reads)
    motor.getPosition().setUpdateFrequency(50);   // 50 Hz
    motor.getVelocity().setUpdateFrequency(50);   // 50 Hz
    motor.getStatorCurrent().setUpdateFrequency(10);  // 10 Hz (less critical)

    // Optimize CAN bus usage
    motor.optimizeBusUtilization();
}
```

---

## Monitoring Performance

### With Epilogue

Epilogue logs its own performance to `/Epilogue/Stats/Last Run` in NetworkTables. View this in AdvantageScope to see how long logging takes each cycle.

### Manual Timing

The codebase already has `TimingUtils.logDuration()`. Use it to profile specific sections:

```java
@Override
public void periodic() {
    TimingUtils.logDuration("ShooterSubsystem.control", () -> {
        // control logic
    });

    TimingUtils.logDuration("ShooterSubsystem.telemetry", () -> {
        // telemetry (if not using Epilogue)
    });
}
```

---

## Summary

| Approach | Effort | Benefit | Recommended For |
|----------|--------|---------|-----------------|
| **Epilogue** | Medium | High - automatic, optimized | Long-term solution |
| **Throttling + Key Caching** | Low | Medium - quick improvement | Immediate fix |
| **AdvantageKit** | High | Highest - full replay | Off-season project |
| **DogLog** | Low | Medium - simple API | Teams wanting simplicity |

**Recommendation:** Migrate to Epilogue and cache CAN reads.

---

## Sources

- [WPILib: Robot Telemetry with Annotations](https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html)