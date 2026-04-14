package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;
import frc.robot.RobotContainer;

@Logged
public class FloorFeedSubsystem extends SubsystemBase {

    // If you want sawtooth, top and bottom speeds need to be different. ex: 100 and 70. Currently disabled.
    private final double motorSpeedTop = 100; // in rps
    // private final double motorSpeedBottom = 100; 

    private boolean isFeeding = false;
    private boolean isOuttaking = false;
    private double secondsToHighPoint = 0.001;
    private double secondsToLowPoint = 1.25;
    private double speedChangeFactor = 1.0;
    @NotLogged
    private Timer delayTimer = new Timer();
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
    public ShooterSubsystem shooterSubsystem;
    @NotLogged
    public RobotContainer robotContainer;
    private int statorCurrentLimit;
    private int supplyCurrentLimit;
    private int lowStatorCurrentLimit;
    private int lowSupplyCurrentLimit;

    public FloorFeedSubsystem(int statorCurrentLimit, int supplyCurrentLimit, int lowStatorCurrentLimit, int lowSupplyCurrentLimit, ShooterSubsystem shooterSubsystem, RobotContainer robotContainer) {
        this.statorCurrentLimit = statorCurrentLimit;
        this.supplyCurrentLimit = supplyCurrentLimit;
        this.lowStatorCurrentLimit = lowStatorCurrentLimit;
        this.lowSupplyCurrentLimit = lowSupplyCurrentLimit;
        this.shooterSubsystem = shooterSubsystem;
        this.robotContainer = robotContainer;
        motor = PIDMotor.makeMotor(Constants.FLOOR_FEED_MOTOR_ID, "Floor Feed",
                PARAM_P, PARAM_I, PARAM_D, PARAM_S, PARAM_V, PARAM_A, PARAM_MV, PARAM_MA, PARAM_MJ);
        motor.setInverted(InvertedValue.Clockwise_Positive);
        motor.setStatorCurrentLimit(statorCurrentLimit);
        motor.setSupplyCurrentLimit(supplyCurrentLimit);
        motor.setIdleCoastMode();
    }

    public boolean getIsFeeding() {
        return isFeeding;
    }
    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    public boolean getIsOuttaking() {
        return isOuttaking;
    }
    public void setIsOuttaking(boolean outtaking) {
        isOuttaking = outtaking;
    }

    public boolean getNeedToRun() {
        return needsToRun;
    }

    public double getCurrentSpeed() {
        return motor.getVelocity();
    }

    public void updateParameters(){
        motor.fetchPIDFFromDashboard();
        this.secondsToHighPoint = SmartDashboard.getNumber("secondsToHighPoint", secondsToHighPoint);
        this.secondsToLowPoint = SmartDashboard.getNumber("secondsToLowPoint", secondsToLowPoint);
        this.speedChangeFactor = SmartDashboard.getNumber("speedChangeFactor", speedChangeFactor);
    }

    private double getVelocityAtTime(double t) {

        return motorSpeedTop;

        // double highSlope, lowSlope, speedDelta;

        // // find parameters
        // speedDelta = motorSpeedTop - motorSpeedBottom;

        // highSlope = speedDelta / secondsToHighPoint;
        // lowSlope = -(speedDelta / secondsToLowPoint);

        // double x = t % (secondsToHighPoint + secondsToLowPoint);
        // // find value
        // if (x < secondsToHighPoint) {
        //     // upwards slope
        //     return highSlope * x + motorSpeedBottom;
        // } else {
        //     // downwards slope
        //     return lowSlope * (x - secondsToHighPoint) + motorSpeedTop;
        // }

    }

    @Override
    public void periodic() {
        if (!needsToRun && shooterSubsystem.needsFloorFeed() && !delayTimer.isRunning()) {
            delayTimer.reset();
            delayTimer.start();
        }

        if (shooterSubsystem.needsFloorFeed() && delayTimer.hasElapsed(0.1)) {
            needsToRun = true;
            delayTimer.stop();
        }
        else {
            needsToRun = false;
        }

        if (isOuttaking) {
            motor.setVelocityTarget(-getVelocityAtTime(Timer.getFPGATimestamp()));
        } else if (needsToRun && this.robotContainer.powerSavingState.priority < RobotContainer.PowerSavingState.NO_FLOOR.priority) {
            // Use FPGA timestamp for consistent timing
            motor.setVelocityTarget(getVelocityAtTime(Timer.getFPGATimestamp()));
        } else {
            motor.setPercentOutput(0);
        }
    }

    public void setLowCurrentMode(boolean lowCurrent) {
        if (lowCurrent) {
            motor.setStatorCurrentLimit(this.lowStatorCurrentLimit);
            motor.setSupplyCurrentLimit(this.lowSupplyCurrentLimit);
        } else {
            motor.setStatorCurrentLimit(this.statorCurrentLimit);
            motor.setSupplyCurrentLimit(this.supplyCurrentLimit);
        }
    }

}
