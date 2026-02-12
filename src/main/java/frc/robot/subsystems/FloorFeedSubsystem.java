package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class FloorFeedSubsystem extends SubsystemBase {
    
    /**
     * Call updateTargets() on change.
     * Actual speed will oscillate around this.
     */
    private double motorSpeedCentre = 10; // in rps. i just randomly picked this value. TODO TUNE THIS
    private boolean isFeeding = false;

    private PIDMotor motor;
    
    private double lowHighAccel, highLowAccel;
    private double lowPoint, highPoint;
    private boolean isAccelerating = true;
    // force a retarget, i.e. reset velocity targets
    private boolean forceRetarget = false;

    private static final double MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps
    private static final double RPS_STEP = 4.0; // rps

    private static final double SECONDS_TO_HIGH_POINT = 0.5; // time from min to max speed in seconds  
    private static final double TIME_FACTOR_TO_LOW_POINT = 3; // how much longer high-to-low takes (factor)
    private static final double SPEED_CHANGE_FACTOR = 0.5; // total height of the wave, as a factor of the setpoint (50% means 75-125%)
    private static final double TOLERANCE_FACTOR = 0.05; // tolerance to consider the target hit (5%)

    // have to be constants for the periodic acceleration changes
    private static final double PARAM_P = 0.11;
    private static final double PARAM_I = 0.0;
    private static final double PARAM_D = 0.0;
    private static final double PARAM_S = 0.25;
    private static final double PARAM_V = 1.2;
    private static final double PARAM_A = 0.01; /* changed */
    private static final double PARAM_MV = MAX_RPS;
    private static final double PARAM_MA = MAX_RPS * 2;
    private static final double PARAM_MJ = 0.00;

    public FloorFeedSubsystem(int currentLimit) {
        motor = PIDMotor.makeMotor(Constants.FLOOR_FEED_MOTOR_ID, "Floor Feed",
            PARAM_P, PARAM_I, PARAM_D, PARAM_S, PARAM_V, PARAM_A, PARAM_MV, PARAM_MA, PARAM_MJ);
        motor.setCurrentLimit(currentLimit);
        motor.setIdleCoastMode();
    }

    /**
     * Based on the currently-set motor speed centre, update setpoints and
     * acceleration values. Also forces a retarget of the motor (re-set the
     * speed setpoint outside of an oscillation target hit).
     */
    private void updateTargets() {
        lowHighAccel = (1 / SECONDS_TO_HIGH_POINT) * SPEED_CHANGE_FACTOR * motorSpeedCentre; // in rps/s
        highLowAccel = lowHighAccel * TIME_FACTOR_TO_LOW_POINT;
        lowPoint = motorSpeedCentre * (1 - SPEED_CHANGE_FACTOR / 2);
        highPoint = motorSpeedCentre * (1 + SPEED_CHANGE_FACTOR / 2);
        this.forceRetarget = true;
    }

    public boolean getIsFeeding() {
        return isFeeding;
    }
    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    public double getMotorSpeedCentre() {
        return motorSpeedCentre;
    }
    public void setMotorSpeedCentre(double speed) {
        motorSpeedCentre = speed;
        this.updateTargets();
    }

    public void incrementMotorSpeed() {
        motorSpeedCentre += RPS_STEP;
        motorSpeedCentre = Math.min(motorSpeedCentre, MAX_RPS);
        this.updateTargets();
    }
    public void decrementMotorSpeed() {
        motorSpeedCentre -= RPS_STEP;
        motorSpeedCentre = Math.max(motorSpeedCentre, -MAX_RPS);
        this.updateTargets();
    }

    @Override
    public void periodic() {
        
        this.motorSpeedCentre = SmartDashboard.getNumber("Floor Feed motor speed centre", this.motorSpeedCentre);

        // This method will be called once per scheduler run
        if (isFeeding) {

            if (isAccelerating) { // targeting high point

                // set a new velocity if requested
                if (this.forceRetarget) {
                    motor.setVelocityTarget(highPoint);
                    this.forceRetarget = false;
                }

                // check if the target has been reached
                if (motor.getVelocity() > highPoint * (1 - TOLERANCE_FACTOR)) {
                    // swap parameters
                    motor.setPIDF(PARAM_P, PARAM_I, PARAM_D, PARAM_S, PARAM_V, 
                        highLowAccel /* A */, PARAM_MV, highLowAccel /* max A */, PARAM_MJ);
                    motor.setVelocityTarget(lowPoint);
                    isAccelerating = false;
                }

                // otherwise wait

            } else { // targeting low point

                // set a new velocity if requested
                if (this.forceRetarget) {
                    motor.setVelocityTarget(lowPoint);
                    this.forceRetarget = false;
                }

                // check if the target has been reached
                if (motor.getVelocity() > lowPoint * (1 + TOLERANCE_FACTOR)) {
                    // swap parameters
                    motor.setPIDF(PARAM_P, PARAM_I, PARAM_D, PARAM_S, PARAM_V, 
                        lowHighAccel /* A */, PARAM_MV, lowHighAccel /* max A */, PARAM_MJ);
                    motor.setVelocityTarget(highPoint);
                    isAccelerating = true;
                }

                // otherwise wait

            }

        } else {
            // kill motor
            motor.setPercentOutput(0);
        }

        SmartDashboard.putBoolean("Floor Feed is feeding", isFeeding);
        SmartDashboard.putNumber("Floor Feed motor speed centre", motorSpeedCentre);
        SmartDashboard.putNumber("Floor Feed motor real speed", motor.getVelocity());
    }

}
