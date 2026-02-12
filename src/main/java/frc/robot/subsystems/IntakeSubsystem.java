package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

public class IntakeSubsystem extends SubsystemBase {
    private boolean isIntaking = false;
    private double intakeMotorSpeed = 84; // in rps
    private double tiltPositionAbsolute = 0.0; // 0.0 to 1.0
    public PIDMotor intakePIDMotor;
    public PIDMotor tiltPIDMotor;

    private double tiltMaxSpeed = 140.0;
    private double tiltMaxAccel = 140.0 / 5.0;

    /**
     * The maximum rotational speed of the intake motor, in rotations per
     * second.
     */
    private static final double INTAKE_MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps
    /**
     * The absolute maximum rotational speed of the tilt motor, in rotations
     * per second. There is a separate field that can be modified on the fly,
     * but the speed will never exceed this value.
     */
    private static final double TILT_ABSOLUTE_MAX_RPS = 140.0; // past motor's physical limits
    /**
     * The absolute maximum acceleration of the tilt motor, in rotations
     * per second squared. There is a separate field that can be modified on
     * the fly, but the acceleration will never exceed this value.
     */
    private static final double TILT_ABSOLUTE_MAX_ACCEL = TILT_ABSOLUTE_MAX_RPS * 2; // max 500ms startup? This might be too restrictive but that is unknown until testing
    /**
     * The step size used by the increment and decrement functions.
     */
    private static final double INTAKE_RPS_STEP = 4.0; // rps

    // Functions:
    // 1. Feed - boolean, speed profile
    // 2. Tilt - servo

    /**
     * Creates a new IntakeSubsystem, using the given motor IDs for the intake
     * (i.e. fuel movement) and tilt functions. The tilt motor is driven to an
     * absolute position, where 0.0 is the "home" position and 1.0 is exactly
     * one rotation away from home.
     * @param intakeMotorID The motor ID for the fuel-moving intake motor.
     * @param tiltMotorID The motor ID for the tilt motor.
     */
    public IntakeSubsystem(int intakeMotorID, int tiltMotorID, int intakeCurrentLimit, int tiltCurrentLimit) {
        intakePIDMotor = PIDMotor.makeMotor(intakeMotorID, "intake", 0.11, 0.0, 0.0,
                0.25, 1.2, 0.01, INTAKE_MAX_RPS, INTAKE_MAX_RPS / 5, 0.00);
        intakePIDMotor.setCurrentLimit(intakeCurrentLimit);
        intakePIDMotor.setIdleCoastMode();

        tiltPIDMotor = PIDMotor.makeMotor(tiltMotorID, "tilt", 0.11, 0.0, 0.0,
                0.25, 1.2, 0.01, TILT_ABSOLUTE_MAX_RPS, TILT_ABSOLUTE_MAX_ACCEL, 0.00);
        tiltPIDMotor.setCurrentLimit(tiltCurrentLimit);
        tiltPIDMotor.setIdleCoastMode();

        this.intakeMotorSpeed = intakePIDMotor.getPosition();
    }

    /**
     * Queries whether this intake is currently being instructed to run the
     * intake motor. The intake motor is automatically stopped on disable.
     * @return true if the intake motor is running, false otherwise
     */
    public boolean getIsIntaking() {
        return isIntaking;
    }
    /**
     * Set the status of the intake motor, where true means the intake motor
     * should run, and false means it should coast to a stop.
     * @param intaking true to run the intake motor, false to stop running it
     */
    public void setIsIntaking(boolean intaking) {
        isIntaking = intaking;
    }
    /**
     * Get the currently requested intake motor speed, in rotations per second.
     * A negative speed indicates running the motor in reverse.
     * @return the intake motor speed in rotations per second
     */
    public double getIntakeSpeed() {
        return intakeMotorSpeed;
    }
    /**
     * Request the intake motor to run at the given speed, in rotations per
     * second. A negative speed indicates running the motor in reverse.
     * @param speed the intake motor speed in rotations per second
     */
    public void setIntakeSpeed(double speed) {
        intakeMotorSpeed = speed;
    }

    /**
     * Increment the current intake motor speed by some constant step,
     * specified by {@code INTAKE_RPS_STEP}. The speed will never exceed
     * {@code INTAKE_MAX_RPS}.
     */
    public void incrementIntakeSpeed() {
        intakeMotorSpeed = ExtraMath.clamp(intakeMotorSpeed + INTAKE_RPS_STEP, -INTAKE_MAX_RPS, INTAKE_MAX_RPS);
    }
    /**
     * Decrement the current intake motor speed by some constant step,
     * specified by {@code INTAKE_RPS_STEP}. The speed will never exceed
     * {@code -INTAKE_MAX_RPS}.
     */
    public void decrementIntakeSpeed() {
        intakeMotorSpeed = ExtraMath.clamp(intakeMotorSpeed - INTAKE_RPS_STEP, -INTAKE_MAX_RPS, INTAKE_MAX_RPS);
    }

    /**
     * Get the currently requested absolute tilt position, in full rotations.
     * @return the tilt position in full rotations
     */
    public double getTiltPosition() {
        return tiltPositionAbsolute;
    }
    /**
     * Set the target absolute tilt position, in full rotations.
     * @param position the tilt position in full rotations
     */
    public void setTiltPosition(double position) { // We want an Up, Middle and Down preset
        tiltPositionAbsolute = position;
        tiltPIDMotor.setTarget(tiltPositionAbsolute, tiltMaxSpeed, tiltMaxAccel);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Intake Speed", intakeMotorSpeed);
        SmartDashboard.putBoolean("Is Intaking", isIntaking);
        SmartDashboard.putNumber("Intake Actual Speed", intakePIDMotor.getVelocity());

        SmartDashboard.putNumber("Tilt Position Target", tiltPositionAbsolute);
        SmartDashboard.putNumber("Tilt Actual Position", tiltPIDMotor.getPosition());

        isIntaking = SmartDashboard.getBoolean("Is Intaking", isIntaking);

        if (isIntaking)
            intakePIDMotor.setVelocityTarget(intakeMotorSpeed);
        else {
            intakePIDMotor.setPercentOutput(0);
        }

    }

}
