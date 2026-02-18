package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDMotor;

@Logged
public class IntakeSubsystem extends SubsystemBase {

    // Logged automatically by Epilogue
    private boolean isIntaking = false;
    private double tiltPositionAbsolute = 0.0; // 0.0 to 1.0

    @Logged(name = "IntakeMotor")
    public PIDMotor intakePIDMotor;
    @Logged(name = "TiltMotor")
    public PIDMotor tiltPIDMotor;

    @NotLogged
    private double tiltMaxSpeed = 140.0;
    @NotLogged
    private double tiltMaxAccel = 140.0 / 5.0;

    @NotLogged
    private static final double TILT_ABSOLUTE_MAX_RPS = 140.0;
    @NotLogged
    private static final double TILT_ABSOLUTE_MAX_ACCEL = TILT_ABSOLUTE_MAX_RPS * 2;
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

        tiltPIDMotor = PIDMotor.makeMotor(tiltMotorID, "tilt", 0.11, 0.0, 0.0,
                0.25, 1.2, 0.01, TILT_ABSOLUTE_MAX_RPS, TILT_ABSOLUTE_MAX_ACCEL, 0.00);
        tiltPIDMotor.setCurrentLimit(tiltCurrentLimit);
        tiltPIDMotor.setIdleCoastMode();
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
        return tiltPositionAbsolute;
    }

    /**
     * Set the target absolute tilt position, in full rotations.
     * 
     * @param position the tilt position in full rotations
     */
    public void setTiltPosition(double position) { // We want an Up, Middle and Down preset
        // tiltPositionAbsolute = position;
        // tiltPIDMotor.setTarget(tiltPositionAbsolute, tiltMaxSpeed, tiltMaxAccel);
    }

    public void updateParameters() {
        // TODO: tilt motor
    }

    @Override
    public void periodic() {
        if (isIntaking) {
            intakePIDMotor.setPercentOutput(1);
        } else if(floorSubsystem.getNeedToRun()){
            intakePIDMotor.setVelocityTarget(floorSubsystem.getCurrentSpeed());
        } else {
            intakePIDMotor.setPercentOutput(0);
        }
    }

}
