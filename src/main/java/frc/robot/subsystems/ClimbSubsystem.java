package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ClimbSubsystem {
    
    private boolean isExtended = false;
    private PIDMotor climbMotor;

    private static final double EXTENDED_POSITION = 1.0;
    private static final double RETRACTED_POSITION = 0.0;
    private static final double MAX_VELOCITY = 140.0;

    public ClimbSubsystem(int currentLimit) {
        // only driven in abs position mode
        climbMotor = PIDMotor.makeMotor(Constants.CLIMB_MOTOR_ID, "Climb", 0.11, 0.0, 0.0,
                0.25, 1.2, 0.01, MAX_VELOCITY, MAX_VELOCITY / 5, 0.00);
        climbMotor.setCurrentLimit(currentLimit);
        climbMotor.setIdleBrakeMode();
    }

    public boolean getIsExtended() {
        return isExtended;
    }
    
    public void extend() {
        this.climbMotor.setTarget(EXTENDED_POSITION);
        this.isExtended = true;
    }
    public void retract() {
        this.climbMotor.setTarget(RETRACTED_POSITION);
        this.isExtended = false;
    }

}
