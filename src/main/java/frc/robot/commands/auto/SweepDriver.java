package frc.robot.commands.auto;

import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class SweepDriver extends AutoCommand {

    private PathPlannerPath pathRight;
    private PathPlannerPath pathLeft;
    private CommandSwerveDrivetrain drivetrain;
    
    public SweepDriver(RobotContainer robotContainer, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        pathRight = AutoBuildingBlocks.loadPathOrThrow("SweepDriverRight.1");
        pathLeft = AutoBuildingBlocks.loadPathOrThrow("SweepDriverLeft.1");
        this.drivetrain = drivetrain;

        addCommands(
            new WaitCommand(0.01),
            AutoBuildingBlocks.autoStep("PATH"),
            Commands.race(
                drivetrain.applyRequest(() -> robotContainer.driveToPose(getProperFlippedStartingPose(), 2, 6.0)),
                new WaitUntilCommand(() -> robotContainer.atTargetPos(getProperFlippedStartingPose(), 0.1))
            ),
            new DeferredCommand(() -> AutoBuildingBlocks.followPathCommand(getPath(), drivetrain), Set.of(drivetrain))
        );
    }

    private PathPlannerPath getPath() {
        return isOnRightSide(drivetrain) ? pathRight : pathLeft;
    }

    /**
    * Checks if the robot is on the right side of the field from driver perspective
    */
    public boolean isOnRightSide(CommandSwerveDrivetrain drivetrain) {
        double robotY = drivetrain.getState().Pose.getY();
        boolean isBlueAlliance = Robot.alliance == DriverStation.Alliance.Blue;
        
        // Blue alliance: Y < 4 is right side, Red alliance: Y > 4 is right side
        return isBlueAlliance ? (robotY < 4) : (robotY > 4);
    }

    @Override
    List<Pose2d> getAllRawPathPoses() {
        return getPath().getPathPoses();
    }

    @Override
    Pose2d getRawStartingPose() {
        return getPath().getStartingHolonomicPose().orElseThrow();
    }

}
