package frc.robot.commands.auto;

import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootAtuo;
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
            AutoBuildingBlocks.autoStep("PATH 1 AND AIM"),
            
            // Run the path and the aiming/shooting logic simultaneously.
            // The deadline is the path; once the path is done, AutoAimAndShoot ends.
            Commands.deadline(
                AutoBuildingBlocks.followPathCommand(path1, drivetrain),
                new ShootAtuo(robot)
            ),

            AutoBuildingBlocks.autoStep("Intake Sequence 1"),
            Commands.sequence(
                new WaitCommand(4), // Wait before intake up
                new InstantCommand(() -> robot.intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltHalfExtensionPos)),
                new WaitCommand(2.5) // Wait to finish shooting
            ),
            
            AutoBuildingBlocks.autoStep("PATH 2 AND AIM"),
            Commands.deadline(
                AutoBuildingBlocks.followPathCommand(path2, drivetrain),
                new ShootAtuo(robot)
            ),

            AutoBuildingBlocks.autoStep("Intake Sequence 2"),
            Commands.sequence(
                new WaitCommand(4), 
                new InstantCommand(() -> robot.intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltHalfExtensionPos)),
                new WaitCommand(2.5) 
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