package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

public class FeederSubsystem extends SubsystemBase {
    private boolean isFeeding = false;
    private double feederSpeed = 100; // in rps
    private double feederSpeedFactor; // from -1.0 to 1.0
    private boolean velocityMode = true;
    public PIDMotor feedPIDMotor;

    private static final double RPS_STEP = 4.0; // rps
    private static final double INCREMENT_FACTOR = 0.05; // 5 percent
    private static final double MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps

    public FeederSubsystem() {
        // These numbers are placeholders, we don't actually know what they should be yet
        feedPIDMotor = PIDMotor.makeMotor(Constants.FEEDER_MOTOR_ID, "feeder", 1.0, 0.0, 0.1,
                0.25, 0.1, 0.01, 100.0, 300.0, 0.00);
        feedPIDMotor.setCurrentLimit(60);
        feedPIDMotor.setIdleCoastMode();
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

    /**
     * will increase speed by the constant value INCREMENT_FACTOR or RPS_STEP up to max of MAX_RPS
     */
    public void incrementFeedingSpeed() {
        if (velocityMode) {
            feederSpeed = ExtraMath.clamp(feederSpeed + RPS_STEP, -MAX_RPS, MAX_RPS);
        }
        else {
            feederSpeedFactor = ExtraMath.clamp(feederSpeedFactor + INCREMENT_FACTOR, -1, 1);
        }
    }
    
    /**
     * will decrease speed by the constant value INCREMENT_FACTOR or RPS_STEP up to max of MAX_RPS
     */
    public void decrementFeedingSpeed() {
        if (velocityMode) {
            feederSpeed = ExtraMath.clamp(feederSpeed - RPS_STEP, -MAX_RPS, MAX_RPS);
        }
        else {
            feederSpeedFactor = ExtraMath.clamp(feederSpeedFactor - INCREMENT_FACTOR, -1, 1);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Speed", feederSpeed);
        SmartDashboard.putNumber("Feeder Percentage", feederSpeedFactor);
        SmartDashboard.putBoolean("Feeder VelocityMode", velocityMode);
        SmartDashboard.putBoolean("Is Feeding", isFeeding);
        SmartDashboard.putNumber("Feeder Actual Speed", feedPIDMotor.getVelocity());

        isFeeding = SmartDashboard.getBoolean("Is Feeding", isFeeding);
        velocityMode = SmartDashboard.getBoolean("Feeder VelocityMode", velocityMode);

        if (velocityMode) {
            if (isFeeding)
                feedPIDMotor.setVelocityTarget(feederSpeed);
            else
                feedPIDMotor.setPercentOutput(0);
        } else {
            if (isFeeding)
                // factor is -1 to 1, converted to a factor of MAX_RPS max speed
                feedPIDMotor.setVelocityTarget(feederSpeedFactor * MAX_RPS); 
            else
                feedPIDMotor.setPercentOutput(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
