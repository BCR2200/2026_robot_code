package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootAtuo extends Command {
    RobotContainer rc;

    public ShootAtuo(RobotContainer rc) {
        this.rc = rc;
        // CRITICAL: We only require the shooter. 
        // This allows PathPlanner to keep control of the drivetrain.
        addRequirements(rc.shooterSubsystem);
    }

    @Override
    public void execute() {
        // This tells the PathPlanner override (from Step 1) to take over rotation
        rc.shootingAtHub = true; 
        rc.passing = false;

        rc.shooterSubsystem.setIsShooting(true);
        if (rc.shooterSubsystem.isShooterAtSpeed()) {
            rc.shooterSubsystem.setIsFeeding(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        rc.shootingAtHub = false;
        rc.shooterSubsystem.setIsShooting(false);
        rc.shooterSubsystem.setIsFeeding(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}