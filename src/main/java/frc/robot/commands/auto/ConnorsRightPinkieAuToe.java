package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class ConnorsRightPinkieAuToe extends AutoCommand{
    private final PathPlannerPath path1;

    public ConnorsRightPinkieAuToe (RobotContainer robot, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        path1 = AutoBuildingBlocks.loadPathOrThrow("RightBump1");
        addCommands(
          new WaitCommand(0.01),
          AutoBuildingBlocks.autoStep("PATH 1"),
          AutoBuildingBlocks.followPathCommand(path1,drivetrain),
          new WaitCommand(10)
          //TODO Add climb command
        );
    }

    
  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream
        .of(path1.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }
}
