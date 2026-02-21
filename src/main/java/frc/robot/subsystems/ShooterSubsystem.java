package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;
import frc.robot.Interpolator;
import frc.robot.LinearActuator;

@Logged
public class ShooterSubsystem extends SubsystemBase {

    @NotLogged
    private static final double PRELOAD_SPEED_PERCENT = 0.5;

    @NotLogged
    private static final double RPS_STEP = 4.0; // rps
    @NotLogged
    private static final double MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps

    // Logged automatically by Epilogue
    private final String name;
    private boolean isShooting = false;
    private double shooterSpeed = 54; // in rps. TODO: remove and use the shooterVelocityInterpolator instead
    private boolean isPassing = false;
    private static final double PASSING_SHOOTER_SPEED = 42; // in rps
    private boolean isFeeding = false;
    private double feederSpeed = 100; //in rps

    // Logged via PIDMotorLogger
    @Logged(name = "ShootMotor")
    public PIDMotor shootPIDMotor;
    @Logged(name = "FeedMotor")
    public PIDMotor feedPIDMotor;

    @NotLogged
    public LinearActuator linearActuator;

    /**
     * Sensor for the beam break.
     * Returns {@code true} if the beam is unbroken, {@code false} if something is present.
     */
    @NotLogged
    private final DigitalInput breamBake; // No emojis (encoding errors)

    @NotLogged
    private final Interpolator shooterAngleInterpolator;
    @NotLogged
    private final Interpolator shooterVelocityInterpolator;

    public ShooterSubsystem(String name, int shooterMotorID, int feederMotorID, int beambreakChannel, int actuatorChannel, int shootCurrentLimit, int feedCurrentLimit, 
                            Interpolator shooterAngleInterpolator, Interpolator shooterVelocityInterpolator, boolean isMountedIncorrectly) {
        this.name = name;
        breamBake = new DigitalInput(beambreakChannel);
        
                                // These numbers are placeholders, we don't actually know what they should be yet
        shootPIDMotor = PIDMotor.makeMotor(shooterMotorID, name + " shooter", 0.1, 0.0, 0.0,
                0.2, 0.0957, 0.0, MAX_RPS, MAX_RPS, 0.00);
        shootPIDMotor.setInverted(isMountedIncorrectly ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        shootPIDMotor.setCurrentLimit(shootCurrentLimit);
        shootPIDMotor.setIdleCoastMode();

        feedPIDMotor = PIDMotor.makeMotor(feederMotorID, name + " feeder", 0.10, 0.0, 0.0,
                0.25, 0.1, 100.0, MAX_RPS, MAX_RPS*10, 0.00);
        feedPIDMotor.setCurrentLimit(feedCurrentLimit);
        feedPIDMotor.setIdleBrakeMode();

        this.shooterAngleInterpolator = shooterAngleInterpolator;
        this.shooterVelocityInterpolator = shooterVelocityInterpolator;
        this.linearActuator = new LinearActuator(actuatorChannel, name + " linear actuator");
        setActuatorTargetPosition(0.35d);
        shootPIDMotor.putPIDF();
    }

    public boolean getIsShooting() {
        return isShooting;
    }
    public void setIsShooting(boolean shooting) {
        isShooting = shooting;
    }
    public boolean getIsPassing() {
        return isPassing;
    }
    public void setIsPassing(boolean passing) {
        isPassing = passing;
    }
    public boolean getIsFeeding() {
        return isFeeding;
    }
    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    /**
     * will increase speed by the constant value RPS_STEP up to max of MAX_RPS
     */
    public void incrementShooterSpeed() {
        shooterSpeed = ExtraMath.clamp(shooterSpeed + RPS_STEP, -MAX_RPS, MAX_RPS);
    }
    /**
     * will decrease speed by the constant value RPS_STEP up to max of MAX_RPS
     */
    public void decrementShooterSpeed() {
        shooterSpeed = ExtraMath.clamp(shooterSpeed - RPS_STEP, -MAX_RPS, MAX_RPS);
    }

    /**
     * will increase speed by the constant value RPS_STEP up to max of MAX_RPS
     */
    public void incrementFeederSpeed() {
        feederSpeed = ExtraMath.clamp(feederSpeed + RPS_STEP, -MAX_RPS, MAX_RPS);
    }
    /**
     * will decrease speed by the constant value RPS_STEP up to max of MAX_RPS
     */
    public void decrementFeederSpeed() {
        feederSpeed = ExtraMath.clamp(feederSpeed - RPS_STEP, -MAX_RPS, MAX_RPS);
    }

    public void setActuatorTargetPosition(double position) {
        linearActuator.setTargetPosition(position);
    }
    @Logged
    public double getActuatorPosition() {
        return linearActuator.getTargetPosition();
    }
    
    public void updateParameters(){
        shooterSpeed = SmartDashboard.getNumber("Shooter Speed", shooterSpeed);
        shootPIDMotor.fetchPIDFFromDashboard();

        feederSpeed = SmartDashboard.getNumber("Feeder Speed", feederSpeed);
        feedPIDMotor.fetchPIDFFromDashboard();
    }

    /**
     * sets the linear actuator to an interpolated passing position based on provided distance 
     * 
     * @param distance the distance from the driver station to the robot
     */
    public void setActuatorToPassPosition(double distance) {
        setActuatorTargetPosition(1);
    }

    /**
     * use this to determine if the shooter is at speed before spinning up the feeder motor
     *
     * @return if the shooter motor is at or above the expected speed
     */
    public boolean isShooterAtSpeed() {
        // Why >5? Because we only set a velocity target in periodic, but isShooterAtSpeed is called when target is still 0,
        // atVelocity() can return true unexpectedly, because its velocity is really 0. 
        return shootPIDMotor.getVelocity() > 5 && shootPIDMotor.atVelocity(3);
    }

    public boolean needsFloorFeed() {
        return isFeeding || !isBeamBroken();
    }

    /**
     * Returns true if beam is broken (ball present).
     * Logged by Epilogue.
     */
    @Logged(name = "BeamBroken")
    public boolean isBeamBroken() {
        return !breamBake.get();
    }

    @Override
    public void periodic() {
        // Control logic only - telemetry handled by Epilogue

        if (isShooting) {
            shootPIDMotor.setVelocityTarget(shooterSpeed); // TODO: use shooterVelocityInterpolator
        } else if (isPassing) {
            shootPIDMotor.setVelocityTarget(PASSING_SHOOTER_SPEED);
        } else {
            shootPIDMotor.setPercentOutput(0);
        }

        // Feed at full speed first,
        // then try to preload (until beam break is broken),
        // then stop
        if (isFeeding) {
            feedPIDMotor.setPercentOutput(1);
        } else if (!isBeamBroken()) {
            feedPIDMotor.setPercentOutput(PRELOAD_SPEED_PERCENT);

        } else {
            feedPIDMotor.setPercentOutput(0);
        }
    }
}
