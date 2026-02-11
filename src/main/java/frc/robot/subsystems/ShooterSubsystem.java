package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;
import frc.robot.Interpolator;
import frc.robot.LinearActuator;

public class ShooterSubsystem extends SubsystemBase {
    private boolean isShooting = false;
    public double shooterSpeed = 84; // in rps

    private boolean isFeeding = false;
    public double feederSpeed = 100; // in rps

    private boolean isPreloading = false;

    public PIDMotor shootPIDMotor;
    public PIDMotor feedPIDMotor;
    public LinearActuator linearActuator;
    
    private Interpolator shooterAngleInterpolator;
    private Interpolator shooterVelocityInterpolator;
    private Interpolator passAngleInterpolator;
    private Interpolator passVelocityInterpolator;
    
    private static final double RPS_STEP = 4.0; // rps
    private static final double MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps

    public ShooterSubsystem(String name, int shooterMotorID, int feederMotorID, int actuatorChannel, 
                            Interpolator shooterAngleInterpolator, Interpolator shooterVelocityInterpolator, 
                            Interpolator passAngleInterpolator, Interpolator passVelocityInterpolator) {
        // These numbers are placeholders, we don't actually know what they should be yet
        shootPIDMotor = PIDMotor.makeMotor(shooterMotorID, name + " shooter", 0.11, 0.0, 0.0,
                0.25, 1.2, 0.01, MAX_RPS, MAX_RPS / 5, 0.00);
        shootPIDMotor.setCurrentLimit(60);
        shootPIDMotor.setIdleCoastMode(); // We want this to be as low as possible while maintaining performance

        feedPIDMotor = PIDMotor.makeMotor(feederMotorID, name + " feeder", 0.11, 0.0, 0.0,
                0.25, 1.2, 0.01, MAX_RPS, MAX_RPS / 5, 0.00);
        feedPIDMotor.setCurrentLimit(60); // Want variable current limits, like 30 for starting, then raise to 60
        feedPIDMotor.setIdleCoastMode();

        this.shooterAngleInterpolator = shooterAngleInterpolator;
        this.shooterVelocityInterpolator = shooterVelocityInterpolator;
        this.linearActuator = new LinearActuator(actuatorChannel, name + " linear actuator");
        this.passAngleInterpolator = passAngleInterpolator;
        this.passVelocityInterpolator = passVelocityInterpolator;
    }

    public boolean getIsShooting() {
        return isShooting;
    }
    public void setIsShooting(boolean shooting) {
        isShooting = shooting;
    }
    public double getShooterSpeed() {
        return shooterSpeed;
    }
    public void setShooterSpeed(double speed) {
        shooterSpeed = speed;
    }

    public boolean getIsFeeding() {
        return isFeeding;
    }
    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }
    public double getFeederSpeed() {
        return feederSpeed;
    }
    public void setFeederSpeed(double speed) {
        feederSpeed = speed;
    }

    public boolean getIsPreloading() {
        return isPreloading;
    }
    public void setIsPreloading(boolean preloading) {
        isPreloading = preloading;
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

    public void setActuatorPosition(double position) {
        linearActuator.setPosition(position);
    }
    public double getActuatorPosition() {
        return linearActuator.getPosition();
    }

    /**
     * sets the linear actuator to an interpolated passing position based on provided distance 
     * 
     * @param distance the distance from the driver station to the robot
     */
    public void setActuatorToPassPosition(double distance) {
        linearActuator.setPosition(passAngleInterpolator.interpolate(distance));
        shootPIDMotor.setVelocityTarget(passVelocityInterpolator.interpolate(distance));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putBoolean("Is Shooting", isShooting);
        SmartDashboard.putNumber("Shooter Actual Speed", shootPIDMotor.getVelocity());
        SmartDashboard.putNumber("Shooter Accel", shootPIDMotor.getAcceleration());

        SmartDashboard.putNumber("Feeder Speed", feederSpeed);
        SmartDashboard.putBoolean("Is Feeding", isFeeding);
        SmartDashboard.putNumber("Feeder Actual Speed", feedPIDMotor.getVelocity());
        SmartDashboard.putNumber("Feeder Accel", feedPIDMotor.getAcceleration());

        isShooting = SmartDashboard.getBoolean("Is Shooting", isShooting);
        isFeeding = SmartDashboard.getBoolean("Is Feeding", isFeeding);

        if (isShooting) {
            shootPIDMotor.setVelocityTarget(shooterSpeed);
        }
        else {
            shootPIDMotor.setPercentOutput(0);
        }

        if (isPreloading) {
            feedPIDMotor.setVelocityTarget(5);
        }
        else if (isFeeding) {
            feedPIDMotor.setVelocityTarget(feederSpeed);
        }
        else {
            feedPIDMotor.setPercentOutput(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
