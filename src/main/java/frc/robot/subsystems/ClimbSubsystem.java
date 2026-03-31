package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

@Logged
public class ClimbSubsystem extends SubsystemBase {

    private boolean isClimbed = false;
    private boolean isZeroed = false;
    // private PIDMotor climbMotor;
    private Timer timer;

    @NotLogged
    private static final double HOME_POSITION = 0.0;
    @NotLogged
    private static final double CLIMB_POSITION = -30.0;
    @NotLogged
    private static final double MAX_VELOCITY = 60.0;
    @NotLogged
    private final int currentLimit;

    public ClimbSubsystem(int initialCurrentLimit, int finalCurrentLimit) {
        // only driven in abs position mode
        // climbMotor = PIDMotor.makeMotor(Constants.CLIMB_MOTOR_ID, "Climb", 0.3, 0.0, 0.0,
                // 0.6, 0.1, 0.0, MAX_VELOCITY, MAX_VELOCITY / 0.25, 0.00);

        // climbMotor.setCurrentLimit(initialCurrentLimit);
        currentLimit = finalCurrentLimit;

        // climbMotor.setIdleBrakeMode();
        timer = new Timer();
    }

    public boolean getIsClimbed() {
        return isClimbed;
    }

    public void updateParameters() {
        // climbMotor.fetchPIDFFromDashboard();
    }

    public void goHome() {
        // this.climbMotor.setTarget(HOME_POSITION);
        this.isClimbed = false;
    }

    public void climb() {
        // this.climbMotor.setTarget(CLIMB_POSITION);
        this.isClimbed = true;
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled() && !isZeroed) {
            if (timer.get() == 0.0) {
                timer.restart();
            }
            if (timer.get() < 5.0) {
                // climbMotor.setPercentOutput(0.2);
            } 
            else if (timer.get() > 5.0) {
                // climbMotor.setPercentOutput(0);
                // climbMotor.setCurrentLimit(currentLimit);
                // climbMotor.resetEncoder();
                isZeroed = true;
            }
        }
    }

}
