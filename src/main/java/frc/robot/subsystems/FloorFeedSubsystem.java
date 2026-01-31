package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class FloorFeedSubsystem extends SubsystemBase {
    
    private boolean isFeeding = false;
    private double motorSpeed = 0; // in rps
    private PIDMotor motor;

    private static final double MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps
    private static final double RPS_STEP = 4.0; // rps

    public FloorFeedSubsystem() {
        motor = PIDMotor.makeMotor(Constants.FLOOR_FEED_MOTOR_ID, "Floor Feed", 0.11, 0.0, 0.0,
                0.25, 1.2, 0.01, MAX_RPS, MAX_RPS / 5, 0.00);
        motor.setCurrentLimit(60);
        motor.setIdleCoastMode();
    }

    public boolean getIsFeeding() {
        return isFeeding;
    }
    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    public double getMotorSpeed() {
        return motorSpeed;
    }
    public void setMotorSpeed(double speed) {
        motorSpeed = speed;
    }

    public void incrementMotorSpeed() {
        motorSpeed += RPS_STEP;
        motorSpeed = Math.min(motorSpeed, MAX_RPS);
    }
    public void decrementMotorSpeed() {
        motorSpeed -= RPS_STEP;
        motorSpeed = Math.max(motorSpeed, -MAX_RPS);
    }

    @Override
    public void periodic() {
        
        this.motorSpeed = SmartDashboard.getNumber("Floor Feed motor speed", this.motorSpeed);

        // This method will be called once per scheduler run
        if (isFeeding) {
            motor.setVelocityTarget(motorSpeed);
        } else {
            motor.setPercentOutput(0);
        }

        SmartDashboard.putBoolean("Floor Feed is feeding", isFeeding);
        SmartDashboard.putNumber("Floor Feed motor speed", motorSpeed);
    }

}
