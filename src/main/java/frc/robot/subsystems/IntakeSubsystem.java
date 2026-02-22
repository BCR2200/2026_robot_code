package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

@Logged
public class IntakeSubsystem extends SubsystemBase {

    // Logged automatically by Epilogue
    private boolean isIntaking = false;
    private boolean isJiggling = false;

    @Logged(name = "IntakeMotor")
    public PIDMotor intakePIDMotor;
    @Logged(name = "TiltMotor")
    public PIDMotor tiltPIDMotor;

    @NotLogged
    private double tiltMaxSpeed = 140.0;
    @NotLogged
    private double tiltMaxAccel = 140.0 / 5.0;

    @NotLogged
    public static final double tiltMaxExtensionPos = -38;
    @NotLogged
    public static final double tiltMinExtensionPos = 0;

    @Logged
    private double tiltPos = 0;

    @NotLogged
    private static final double TILT_ABSOLUTE_MAX_RPS = 140.0;
    @NotLogged
    private static final double TILT_ABSOLUTE_MAX_ACCEL = TILT_ABSOLUTE_MAX_RPS * 2;
    @NotLogged
    private FloorFeedSubsystem floorSubsystem;

    // Functions:
    // 1. Feed - boolean, speed profile
    // 2. Tilt - servo

    /**
     * Creates a new IntakeSubsystem, using the given motor IDs for the intake
     * (i.e. fuel movement) and tilt functions. The tilt motor is driven to an
     * absolute position, where 0.0 is the "home" position and 1.0 is exactly
     * one rotation away from home.
     * 
     * @param intakeMotorID The motor ID for the fuel-moving intake motor.
     * @param tiltMotorID   The motor ID for the tilt motor.
     */
    public IntakeSubsystem(int intakeMotorID, int tiltMotorID, int intakeCurrentLimit, int tiltCurrentLimit,
            FloorFeedSubsystem floorSubsystem) {
        this.floorSubsystem = floorSubsystem;
        intakePIDMotor = PIDMotor.makeMotor(intakeMotorID, "intake", 0.11, 0.0, 0.0,
                0.25, 0.13, 0.01, 100, 1000, 0.00);
        intakePIDMotor.setCurrentLimit(intakeCurrentLimit);
        intakePIDMotor.setIdleCoastMode();

        tiltPIDMotor = PIDMotor.makeMotor(tiltMotorID, "tilt", 0.6, 0.0, 0.0,
                0.25, 0.3, 0.01, TILT_ABSOLUTE_MAX_RPS, TILT_ABSOLUTE_MAX_ACCEL, 0.00);
        tiltPIDMotor.setCurrentLimit(tiltCurrentLimit);
        tiltPIDMotor.setIdleCoastMode();
    }


    /**
     * Set the movement of the intake tilt, where true means the intake 
     * constantly moves/tilts between two points following a sine wave
     * and false means no intake tilt movement.
     * 
     * @param jiggling true to move the intake up and down to promote feeding, false to stop running it
     */
    public void setIsJiggling(boolean jiggling) {
        isJiggling = jiggling;
    }

    /**
     * Queries whether the intake is currently being jiggled in order to 
     * promote fuel/ball consumption/ingestion/feeding.
     * 
     * @return true if the intake is moving up and down to promote feeding, false otherwise
     */
    public boolean getIsJiggling() {
        return isJiggling;
    }

    /**
     * Queries whether this intake is currently being instructed to run the
     * intake motor. The intake motor is automatically stopped on disable.
     * 
     * @return true if the intake motor is running, false otherwise
     */
    public boolean getIsIntaking() {
        return isIntaking;
    }

    /**
     * Set the status of the intake motor, where true means the intake motor
     * should run, and false means it should coast to a stop.
     * 
     * @param intaking true to run the intake motor, false to stop running it
     */
    public void setIsIntaking(boolean intaking) {
        isIntaking = intaking;
    }

    /**
     * Get the currently requested absolute tilt position, in full rotations.
     * 
     * @return the tilt position in full rotations
     */
    public double getTiltPosition() {
        return tiltPIDMotor.getPosition();
    }

    /**
     * Set the target tilt position, in full rotations.
     * 
     * @param position the tilt position in full rotations
     */
    public void setTiltPosition(double position) { // We want an Up, Middle and Down preset
        tiltPos = ExtraMath.clamp(position, tiltMinExtensionPos, tiltMaxExtensionPos);
        tiltPIDMotor.setTarget(tiltPos, tiltMaxSpeed, tiltMaxAccel);
    }

    public void updateParameters() {
        // TODO: tilt motor
    }

    @Override
    public void periodic() {
        if (isIntaking || floorSubsystem.getNeedToRun()) {
            intakePIDMotor.setPercentOutput(1);
        }
        else {
            intakePIDMotor.setPercentOutput(0);
        }

        // Slightly less Horrible untested garbage jiggle function
        // This can be graphed as: tiltPos + amplitude * sin(speed * time)
        if (isJiggling) {
            double speed = 1.0; // Number of cycles per second
            double amplitude = 2; // If it goes 2 up 2 down from current pos, then amp is 2

            // This moves the intake smoothly up and down with a sin wave
            double targetPos = tiltPos + (Math.sin(Timer.getFPGATimestamp() * speed) * amplitude);
            setTiltPosition(targetPos);
        }
        
    }

}
