package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

public class MTFTFFTHTTSSubsystem extends SubsystemBase {
    private boolean isFeeding = false;
    private double feederSpeed; // in rps
    private double feederPercentage; // it's a percent output from -1.0 to 1.0
    private boolean velocityMode = true;
    public PIDMotor feedPIDMotor;



    private final double incrementRPS = 4.0; // rps
    private final double incrementPercentage = 0.05; // percent

    public MTFTFFTHTTSSubsystem() {
        // These numbers are placeholders, we don't actually know what they should be
        // yet
        feedPIDMotor = PIDMotor.makeMotor(Constants.FEEDER_MOTOR_ID,
                "feeder", 1.0, 0.0, 0.1,
                0.25, 0.1, 0.01,
                100.0, 300.0, 0.00);
        feedPIDMotor.setCurrentLimit(30);
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
     * will increase speed by the constant value incrementRPS and incrementPercentage
     * @param increment
     */
    public void incrementFeedingSpeed() {
        if (velocityMode)
            feederSpeed += incrementRPS;
        else
            feederPercentage = ExtraMath.clamp(feederPercentage + incrementPercentage, -1, 1);
    }
    
    /**
     * will decrease speed by the constant value incrementRPS and incrementPercentage
     * @param increment
     */
    public void decrementFeedingSpeed() {
        if (velocityMode)
            feederSpeed -= incrementRPS;
        else
            feederPercentage = ExtraMath.clamp(feederPercentage - incrementPercentage, -1, 1);
    }

    @Override
    public void periodic() {
        if (velocityMode) {
            if (isFeeding)
                feedPIDMotor.setVelocityTarget(feederSpeed);
            else
                feedPIDMotor.setPercentOutput(0);
        } else {
            if (isFeeding)
                feedPIDMotor.setPercentOutput(feederPercentage);
            else
                feedPIDMotor.setPercentOutput(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
