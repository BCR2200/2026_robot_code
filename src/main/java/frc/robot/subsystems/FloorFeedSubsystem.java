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
    private double motorSpeedCentre = 20; // in rps. i just randomly picked this value. TODO TUNE THIS
    private boolean isFeeding = false;

    private PIDMotor motor;
    
    private static final double MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps
    private static final double RPS_STEP = 4.0; // rps

    private static final double SECONDS_TO_HIGH_POINT = 0.2; // time from min to max speed in seconds  
    private static final double TIME_FACTOR_TO_LOW_POINT = 4; // how much longer high-to-low takes (factor)
    private static final double SPEED_CHANGE_FACTOR = 1.0; // total height of the wave, as a factor of the setpoint (50% means 75-125%)

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
    private ShooterSubsystem[] shooters;

    public FloorFeedSubsystem(int currentLimit, ShooterSubsystem... shooters) {
        this.shooters = shooters;
        motor = PIDMotor.makeMotor(Constants.FLOOR_FEED_MOTOR_ID, "Floor Feed",
        PARAM_P, PARAM_I, PARAM_D, PARAM_S, PARAM_V, PARAM_A, PARAM_MV, PARAM_MA, PARAM_MJ);
        motor.setCurrentLimit(currentLimit);
        motor.setIdleCoastMode();
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
    }

    public void incrementMotorSpeed() {
        motorSpeedCentre += RPS_STEP;
        motorSpeedCentre = Math.min(motorSpeedCentre, MAX_RPS);
    }
    public void decrementMotorSpeed() {
        motorSpeedCentre -= RPS_STEP;
        motorSpeedCentre = Math.max(motorSpeedCentre, -MAX_RPS);
    }

    public void updateParameters(){
        motorSpeedCentre = SmartDashboard.getNumber("Floor Feed motor speed centre", motorSpeedCentre);
        motor.fetchPIDFFromDashboard();
    }

    private double getVelocityAtTime(double t) {
        double highSlope, lowSlope, highPoint, lowPoint, speedDelta;

        // find parameters
        highPoint = motorSpeedCentre * (1 + (SPEED_CHANGE_FACTOR / 2));
        lowPoint = motorSpeedCentre * (1 - (SPEED_CHANGE_FACTOR / 2));
        speedDelta = highPoint - lowPoint;
        
        highSlope = speedDelta / SECONDS_TO_HIGH_POINT;
        lowSlope = -(speedDelta / (SECONDS_TO_HIGH_POINT * TIME_FACTOR_TO_LOW_POINT));

        // low-high takes SECONDS_TO_HIGH_POINT, high-low takes SECONDS_TO_HIGH_POINT * TIME_FACTOR_TO_LOW_POINT
        double x = t % (SECONDS_TO_HIGH_POINT + SECONDS_TO_HIGH_POINT * TIME_FACTOR_TO_LOW_POINT);
        // find value
        if (x < SECONDS_TO_HIGH_POINT) {
            // upwards slope
            return highSlope * x + lowPoint;
        } else {
            // downwards slope
            return lowSlope * (x - SECONDS_TO_HIGH_POINT) + highPoint;
        }

    }

    @Override
    public void periodic() {
        
        // this.motorSpeedCentre = SmartDashboard.getNumber("Floor Feed motor speed centre", this.motorSpeedCentre);

        // check if any shooter needs the floor to run (or if isFeeding is manually set to true)
        // |= (or-equals)
        boolean needsToRun = isFeeding;
        for (int i = 0; i < shooters.length; i++) {
            needsToRun |= shooters[i].needsFloorFeed();
            SmartDashboard.putBoolean("shooter " + i + " needs floor", shooters[i].needsFloorFeed());
        }
        SmartDashboard.putBoolean("needs floor", needsToRun);
        motor.putCurrent();

        if (needsToRun) {
            motor.setVelocityTarget(-getVelocityAtTime(System.currentTimeMillis() / 1_000.0d));
        } else {
            motor.setPercentOutput(0);
        }

        SmartDashboard.putBoolean("Floor Feed is feeding", isFeeding);
        SmartDashboard.putNumber("Floor Feed motor speed centre", motorSpeedCentre);
        SmartDashboard.putNumber("Floor Feed motor real speed", motor.getVelocity());
    }

}
