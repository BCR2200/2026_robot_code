package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

public class ShooterSubsystem extends SubsystemBase {
    private boolean isShooting = false;
    private double shooterSpeed; // in rps
    private double shooterSpeedFactor; // from -1.0 to 1.0
    private boolean velocityMode = true;
    public PIDMotor shootPIDMotor;
    
    private static final double RPS_STEP = 4.0; // rps
    private static final double INCREMENT_FACTOR = 0.05; // 5 percent
    private static final double MAX_RPS = 84.0; // 5000 rpm in rps

    public ShooterSubsystem() {
        // These numbers are placeholders, we don't actually know what they should be yet
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

    /**
     * will increase speed by the constant value INCREMENT_FACTOR or RPS_STEP up to max of MAX_RPS
     */
    public void incrementShooterSpeed() {
        if (velocityMode) {
            shooterSpeed = ExtraMath.clamp(shooterSpeed + RPS_STEP, -MAX_RPS, MAX_RPS);
        }
        else {
            shooterSpeedFactor = ExtraMath.clamp(shooterSpeedFactor + INCREMENT_FACTOR, -1, 1);
        }
    }

    /**
     * will decrease speed by the constant value INCREMENT_FACTOR or RPS_STEP up to max of MAX_RPS
     */
    public void decrementShooterSpeed() {
        if (velocityMode) {
            shooterSpeed = ExtraMath.clamp(shooterSpeed - RPS_STEP, -MAX_RPS, MAX_RPS);
        }
        else {
            shooterSpeedFactor = ExtraMath.clamp(shooterSpeedFactor - INCREMENT_FACTOR, -1, 1); 
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putNumber("Shooter Percentage", shooterSpeedFactor);
        SmartDashboard.putBoolean("Shooter VelocityMode", velocityMode);
        SmartDashboard.putBoolean("Is Shooting", isShooting);
        SmartDashboard.putNumber("Shooter Actual Speed", shootPIDMotor.getVelocity());

        isShooting = SmartDashboard.getBoolean("Is Shooting", isShooting);
        velocityMode = SmartDashboard.getBoolean("Shooter VelocityMode", velocityMode);

        if (velocityMode) {
            if (isShooting)
                shootPIDMotor.setVelocityTarget(shooterSpeed);
            else
                shootPIDMotor.setPercentOutput(0);
        } else {
            if (isShooting)
                // factor is -1 to 1, converted to a factor of MAX_RPS max speed
                shootPIDMotor.setVelocityTarget(shooterSpeedFactor * MAX_RPS);
            else
                shootPIDMotor.setPercentOutput(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
