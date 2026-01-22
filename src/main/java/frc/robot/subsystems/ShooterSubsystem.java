package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

public class ShooterSubsystem extends SubsystemBase {
    private boolean isShooting = false;
    private double shooterSpeed; // in rps
    private double shooterPercentage; // it's a percent output from -1.0 to 1.0
    private boolean velocityMode = true;
    public PIDMotor shootPIDMotor;
    
    private final double incrementRPS = 4.0; // rps
    private final double incrementPercentage = 0.05; // percent

    public ShooterSubsystem() {
        // These numbers are placeholders, we don't actually know what they should be
        // yet
        shootPIDMotor = PIDMotor.makeMotor(Constants.SHOOTER_MOTOR_ID, "shooter", 1.0, 0.0, 0.1,
                0.25, 0.1, 0.01, 100.0, 300.0, 0.00);
        shootPIDMotor.setCurrentLimit(30);
        shootPIDMotor.setIdleCoastMode();
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

    public double getShooterPercentage() {
        return shooterPercentage;
    }

    public void setShooterPercentage(double percentage) {
        shooterPercentage = percentage;
    }
    /**
     * will increase speed by the constant value incrementRPS and incrementPercentage
     * @param increment
     */
    public void incrementShooterSpeed() {
        if (velocityMode)
            shooterSpeed += incrementRPS;
        else
            shooterPercentage = ExtraMath.clamp(shooterPercentage + incrementPercentage, -1, 1);
    }

    /**
     * will decrease speed by the constant value incrementRPS and incrementPercentage
     * @param increment
     */
    public void decrementShooterSpeed() {
        if (velocityMode)
            shooterSpeed -= incrementRPS;
        else
            shooterPercentage = ExtraMath.clamp(shooterPercentage - incrementPercentage, -1, 1);
    }

    @Override
    public void periodic() {
        if (velocityMode) {
            if (isShooting)
                shootPIDMotor.setVelocityTarget(shooterSpeed);
            else
                shootPIDMotor.setPercentOutput(0);
        } else {
            if (isShooting)
                shootPIDMotor.setPercentOutput(shooterPercentage);
            else
                shootPIDMotor.setPercentOutput(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
