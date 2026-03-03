package frc.robot.commands.auto;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class BlenderMode extends Command {

    private final PathPlannerPath blueHub;
    private final PathPlannerPath wallLeftFromBlue;
    private final PathPlannerPath redHub;
    private final PathPlannerPath wallRightFromBlue;

    private Pose2d closestPoint;
    private Pose2d endPoint;
    private double howFarIsClosestPoint = 2200;
    private double comparedPointX;
    private double comparedPointY;
    private double currentPointX;
    private double currentPointY;
    private String closestWall;
    private CommandSwerveDrivetrain drivetrain;

    List<Pose2d> blueHubWaypoints;
    List<Pose2d> wallLFromBPoints;
    List<Pose2d> redHubWaypoints;
    List<Pose2d> wallRFromBPoints;

    public BlenderMode(RobotContainer robot, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        this.drivetrain = drivetrain;

        blueHub = AutoBuildingBlocks.loadPathOrThrow("Blue Hub");
        wallLeftFromBlue = AutoBuildingBlocks.loadPathOrThrow("WallLeftFromBlue");
        redHub = AutoBuildingBlocks.loadPathOrThrow("Red Hub");
        wallRightFromBlue = AutoBuildingBlocks.loadPathOrThrow("WallRightFromBlue");

        blueHubWaypoints = blueHub.getPathPoses();
        wallLFromBPoints = wallLeftFromBlue.getPathPoses();
        redHubWaypoints = redHub.getPathPoses();
        wallRFromBPoints = wallRightFromBlue.getPathPoses();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        for (int x = 0; x < blueHubWaypoints.size(); x++) {
            comparedPointX = blueHubWaypoints.get(x).getX();
            comparedPointY = blueHubWaypoints.get(x).getY();

            currentPointY = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getY();
            currentPointX = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getX();

            if (Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                    + Math.pow(comparedPointY - currentPointY, 2)) < howFarIsClosestPoint) {
                howFarIsClosestPoint = Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                        + Math.pow(comparedPointY - currentPointY, 2));
                closestPoint = blueHubWaypoints.get(x);
                closestWall = "blueHub";
            }
        }

        for (int x = 0; x < wallLFromBPoints.size(); x++) {
            comparedPointX = wallLFromBPoints.get(x).getX();
            comparedPointY = wallLFromBPoints.get(x).getY();

            currentPointY = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getY();
            currentPointX = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getX();

            if (Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                    + Math.pow(comparedPointY - currentPointY, 2)) < howFarIsClosestPoint) {
                howFarIsClosestPoint = Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                        + Math.pow(comparedPointY - currentPointY, 2));
                closestPoint = wallLFromBPoints.get(x);
                closestWall = "wallLeftFromBlue";
            }
        }

        for (int x = 0; x < redHubWaypoints.size(); x++) {
            comparedPointX = redHubWaypoints.get(x).getX();
            comparedPointY = redHubWaypoints.get(x).getY();

            currentPointY = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getY();
            currentPointX = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getX();

            if (Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                    + Math.pow(comparedPointY - currentPointY, 2)) < howFarIsClosestPoint) {
                howFarIsClosestPoint = Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                        + Math.pow(comparedPointY - currentPointY, 2));
                closestPoint = redHubWaypoints.get(x);
                closestWall = "redHub";
            }
        }

        for (int x = 0; x < wallRFromBPoints.size(); x++) {
            comparedPointX = wallRFromBPoints.get(x).getX();
            comparedPointY = wallRFromBPoints.get(x).getY();

            currentPointY = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getY();
            currentPointX = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getX();

            if (Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                    + Math.pow(comparedPointY - currentPointY, 2)) < howFarIsClosestPoint) {
                howFarIsClosestPoint = Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                        + Math.pow(comparedPointY - currentPointY, 2));
                closestPoint = wallRFromBPoints.get(x);
                closestWall = "wallRightFromBlue";
            }
        }

        switch (closestWall) {
            case "blueHub":
                endPoint = blueHubWaypoints.get(blueHubWaypoints.size() - 1);
                break;
            case "wallLeftFromBlue":
                endPoint = wallLFromBPoints.get(wallLFromBPoints.size() - 1);
                break;
            case "redHub":
                endPoint = redHubWaypoints.get(redHubWaypoints.size() - 1);
                break;
            case "wallRightFromBlue":
                endPoint = wallRFromBPoints.get(wallRFromBPoints.size() - 1);
                break;
        }

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not
        // use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME),
                closestPoint,
                endPoint);

        // TODO FOR GRAHAM
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for
                                                                                               // this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
        // You can also use unlimited constraints, only limited by motor torque and
        // nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                      // be null for on-the-fly paths.
                // TODO put case to change the ending angle depending on path to match
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation
                                                                   // here. If using a differential drivetrain, the
                                                                   // rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


}
