package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

@Logged
public class FloorFeedSubsystem extends SubsystemBase {

    // Logged automatically by Epilogue
    private double motorSpeedTop = 100; // in rps
    // If you want sawtooth, top and bottom speeds need to be different. ex: 100 and 70. Currently disabled.
    private double motorSpeedBottom = 100; 
    private boolean isFeeding = false;
    private double secondsToHighPoint = 0.001;
    private double secondsToLowPoint = 1.25;
    private double speedChangeFactor = 1.0;
    @Logged
    private boolean needsToRun = false;

    @Logged(name = "Motor")
    private PIDMotor motor;

    @NotLogged
    private static final double MAX_RPS = 140.0;
    @NotLogged
    private static final double RPS_STEP = 4.0;
    @NotLogged
    private static final double PARAM_P = 0.2;
    @NotLogged
    private static final double PARAM_I = 0.0;
    @NotLogged
    private static final double PARAM_D = 0.0;
    @NotLogged
    private static final double PARAM_S = 0.25;
    @NotLogged
    private static final double PARAM_V = 0.13;
    @NotLogged
    private static final double PARAM_A = 0.01;
    @NotLogged
    private static final double PARAM_MV = MAX_RPS;
    @NotLogged
    private static final double PARAM_MA = MAX_RPS * 2;
    @NotLogged
    private static final double PARAM_MJ = 0.00;

    @NotLogged
    public ShooterSubsystem[] shooters;

    public FloorFeedSubsystem(int currentLimit, ShooterSubsystem... shooters) {
        this.shooters = shooters;
        motor = PIDMotor.makeMotor(Constants.FLOOR_FEED_MOTOR_ID, "Floor Feed",
                PARAM_P, PARAM_I, PARAM_D, PARAM_S, PARAM_V, PARAM_A, PARAM_MV, PARAM_MA, PARAM_MJ);
        motor.setInverted(InvertedValue.Clockwise_Positive);
        motor.setCurrentLimit(currentLimit);
        motor.setIdleCoastMode();
    }

    public boolean getIsFeeding() {
        return isFeeding;
    }
    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    public double getMotorSpeedTop() {
        return motorSpeedTop;
    }
    public void setMotorSpeedTop(double speed) {
        motorSpeedTop = speed;
    }

    public void incrementMotorSpeedTop() {
        motorSpeedTop += RPS_STEP;
        motorSpeedTop = Math.min(motorSpeedTop, MAX_RPS);
    }
    public void decrementMotorSpeedTop() {
        motorSpeedTop -= RPS_STEP;
        motorSpeedTop = Math.max(motorSpeedTop, -MAX_RPS);
    }

    public double getMotorSpeedBottom() {
        return motorSpeedTop;
    }
    public void setMotorSpeedBottom(double speed) {
        motorSpeedBottom = speed;
    }

    public void incrementMotorSpeedBottom() {
        motorSpeedBottom += RPS_STEP;
        motorSpeedBottom = Math.min(motorSpeedBottom, MAX_RPS);
    }
    public void decrementMotorSpeedBottom() {
        motorSpeedBottom -= RPS_STEP;
        motorSpeedBottom = Math.max(motorSpeedBottom, -MAX_RPS);
    }

    public boolean getNeedToRun() {
        return needsToRun;
    }

    public double getCurrentSpeed() {
        return motor.getVelocity();
    }

    public void updateParameters(){
        motorSpeedBottom = SmartDashboard.getNumber("motorSpeedBottom", motorSpeedBottom);
        motorSpeedTop = SmartDashboard.getNumber("motorSpeedTop", motorSpeedTop);
        motor.fetchPIDFFromDashboard();
        this.secondsToHighPoint = SmartDashboard.getNumber("secondsToHighPoint", secondsToHighPoint);
        this.secondsToLowPoint = SmartDashboard.getNumber("secondsToLowPoint", secondsToLowPoint);
        this.speedChangeFactor = SmartDashboard.getNumber("speedChangeFactor", speedChangeFactor);
    }

    private double getVelocityAtTime(double t) {
        double highSlope, lowSlope, speedDelta;

        // find parameters
        speedDelta = motorSpeedTop - motorSpeedBottom;

        highSlope = speedDelta / secondsToHighPoint;
        lowSlope = -(speedDelta / secondsToLowPoint);

        double x = t % (secondsToHighPoint + secondsToLowPoint);
        // find value
        if (x < secondsToHighPoint) {
            // upwards slope
            return highSlope * x + motorSpeedBottom;
        } else {
            // downwards slope
            return lowSlope * (x - secondsToHighPoint) + motorSpeedTop;
        }

    }

    @Override
    public void periodic() {
        // Control logic only - telemetry handled by Epilogue

        // Check if any shooter needs the floor to run (or if isFeeding is manually set to true)
        boolean tmpNeedsToRun = false;
        for (ShooterSubsystem shooter : shooters) {
            tmpNeedsToRun |= shooter.needsFloorFeed();
            if (tmpNeedsToRun) break;
        }
        this.needsToRun = tmpNeedsToRun;

        if (needsToRun) {
            // Use FPGA timestamp for consistent timing
            motor.setVelocityTarget(getVelocityAtTime(Timer.getFPGATimestamp()));
        } else {
            motor.setPercentOutput(0);
        }
    }

}
