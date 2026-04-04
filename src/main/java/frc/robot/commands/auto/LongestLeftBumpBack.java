package frc.robot.commands.auto;

import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootAt;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class LongestLeftBumpBack extends AutoCommand {

    private PathPlannerPath path1;
    private PathPlannerPath path2;
    
    public LongestLeftBumpBack(RobotContainer robot, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        path1 = AutoBuildingBlocks.loadPathOrThrow("LongLeftBumpBack.Connor");
        path2 = AutoBuildingBlocks.loadPathOrThrow("LongestLeftBumpBack.Connor");

        addCommands(
            new WaitCommand(0.01),
            AutoBuildingBlocks.autoStep("PATH 1"),
            AutoBuildingBlocks.followPathCommand(path1, drivetrain),
            AutoBuildingBlocks.autoStep("Shoot 1"),
            Commands.race(
                new ShootAt(robot),
                Commands.sequence(
                    new WaitCommand(4), // Wait before intake up
                    new InstantCommand(() -> robot.intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltHalfExtensionPos)),
                    new WaitCommand(2.5) // Wait to finish shooting
                )
            ),
            
            AutoBuildingBlocks.autoStep("PATH 2"),
            AutoBuildingBlocks.followPathCommand(path2, drivetrain),
            AutoBuildingBlocks.autoStep("Shoot 2"),
            Commands.race(
                new ShootAt(robot),
                Commands.sequence(
                    new WaitCommand(4), // Wait before intake up
                    new InstantCommand(() -> robot.intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltHalfExtensionPos)),
                    new WaitCommand(2.5) // Wait to finish shooting
                )
            )
        );
    }

    @Override
    List<Pose2d> getAllRawPathPoses() {
        return path1.getPathPoses();
    }

    @Override
    Pose2d getRawStartingPose() {
        return path1.getStartingHolonomicPose().orElseThrow();
    }

}
