package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.CommandSwerveDrivetrain;

public class MoveAndShoot extends SequentialCommandGroup {

  public MoveAndShoot (PathPlannerPath path, CommandSwerveDrivetrain drivetrain) {
    addCommands(
      new ParallelCommandGroup(
          AutoBuildingBlocks.followPathCommand(path, drivetrain)
          //TODO Add Shoot CMD and alter parameters for the CMD
        )
    );
  }
}
