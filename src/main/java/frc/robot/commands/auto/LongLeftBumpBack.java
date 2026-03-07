package frc.robot.commands.auto;

import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ShootAt;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class LongLeftBumpBack extends AutoCommand {

    private PathPlannerPath path;
    
    public LongLeftBumpBack(RobotContainer robot, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        path = AutoBuildingBlocks.loadPathOrThrow("LongLeftBumpBack.1");

        addCommands(
            new WaitCommand(0.01),
            AutoBuildingBlocks.autoStep("PATH"),
            AutoBuildingBlocks.followPathCommand(path, drivetrain),
            AutoBuildingBlocks.autoStep("Shoot"),
            Commands.race(
                new ShootAt(robot),
                Commands.sequence(
                    new WaitCommand(4), // Wait before intake up
                    new InstantCommand(() -> robot.intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltHalfExtensionPos)),
                    new WaitCommand(2.5) // Wait to finish shooting
                )
            ),
            new ClimbCommand(robot, false)
        );
    }

    @Override
    List<Pose2d> getAllRawPathPoses() {
        return path.getPathPoses();
    }

    @Override
    Pose2d getRawStartingPose() {
        return path.getStartingHolonomicPose().orElseThrow();
    }

}
