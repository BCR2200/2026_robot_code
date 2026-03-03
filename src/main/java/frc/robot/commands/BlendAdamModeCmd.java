package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.LimelightHelpers;
import frc.robot.OURLimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.AutoBuildingBlocks;
import frc.robot.drive.CommandSwerveDrivetrain;

public class BlendAdamModeCmd extends Command {
    private static enum BlenderWalls {
        BLUE_HUB, WALL_LEFT_FROM_BLUE, RED_HUB, WALL_RIGHT_FROM_BLUE
    };

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
    private BlenderWalls closestWall;
    private CommandSwerveDrivetrain drivetrain;

    List<Pose2d> blueHubWaypoints;
    List<Pose2d> wallLFromBPoints;
    List<Pose2d> redHubWaypoints;
    List<Pose2d> wallRFromBPoints;

    private boolean doneFirstPath = false;
    private Pose2d currentPose2d;

    private double MaxSpeed = RobotContainer.MaxSpeed;
    private double MaxAngularRate = RobotContainer.MaxAngularRate;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandXboxController m_driverController;

    private Distance blueHUBXLine = Distance.ofBaseUnits(245, Units.Inches);
    private Distance redHUBXLine = Distance.ofBaseUnits(457, Units.Inches);
    private Distance wallLFromBYLine = Distance.ofBaseUnits(30, Units.Inches);
    private Distance wallRFromBYLine = Distance.ofBaseUnits(287.69, Units.Inches);

    public BlendAdamModeCmd(RobotContainer robot, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve,
            CommandXboxController m_driverController) {
        this.drivetrain = drivetrain;
        this.m_driverController = m_driverController;

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
        setClosestWallAndPoint(blueHubWaypoints, BlenderWalls.BLUE_HUB);
        setClosestWallAndPoint(wallLFromBPoints, BlenderWalls.WALL_LEFT_FROM_BLUE);
        setClosestWallAndPoint(redHubWaypoints, BlenderWalls.RED_HUB);
        setClosestWallAndPoint(wallRFromBPoints, BlenderWalls.WALL_RIGHT_FROM_BLUE);

        // switch (closestWall) {
        // case BLUE_HUB:
        // endPoint = blueHubWaypoints.get(blueHubWaypoints.size() - 1);
        // break;
        // case WALL_LEFT_FROM_BLUE:
        // endPoint = wallLFromBPoints.get(wallLFromBPoints.size() - 1);
        // break;
        // case RED_HUB:
        // endPoint = redHubWaypoints.get(redHubWaypoints.size() - 1);
        // break;
        // case WALL_RIGHT_FROM_BLUE:
        // endPoint = wallRFromBPoints.get(wallRFromBPoints.size() - 1);
        // break;
        // }

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not
        // use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME),
                closestPoint);

        // TODO FOR GRAHAM https://www.youtube.com/watch?v=cF1Na4AIecM
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

        doneFirstPath = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (doneFirstPath) {
            currentPose2d = OURLimelightHelpers.betterGetPose2d(Constants.FEEDER_LIMELIGHT_NAME,
                    Constants.SHOOTER_LIMELIGHT_NAME);
            /*
             * If (in a corner, allow driver X and Y)
             * If (On one of the X lines, only allow Y)
             * If (On one of the Y lines, only allow X)
             * If (Not on either, free reign)
             */
            if(ExtraMath.within(currentPose2d.getY(), redHUBXLine.in(Units.Meters), 0.5)
                || ExtraMath.within(currentPose2d.getY(), blueHUBXLine.in(Units.Meters), 0.5) && 
                ExtraMath.within(currentPose2d.getX(), wallLFromBYLine.in(Units.Meters), 0.5) 
                || ExtraMath.within(currentPose2d.getX(), wallRFromBYLine.in(Units.Meters), 0.5)){
                    drivetrain.setControl(drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
            }
            else if(ExtraMath.within(currentPose2d.getX(), redHUBXLine.in(Units.Meters), 0.5)
                || ExtraMath.within(currentPose2d.getX(), blueHUBXLine.in(Units.Meters), 0.5)){
                    drivetrain.setControl(drive.withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
            }
            else if(ExtraMath.within(currentPose2d.getY(), wallLFromBYLine.in(Units.Meters), 0.5) 
                || ExtraMath.within(currentPose2d.getY(), wallRFromBYLine.in(Units.Meters), 0.5)){
                    drivetrain.setControl(drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed));
            } else {
                drivetrain.setControl(drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
            } 
        }
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

    public void setClosestWallAndPoint(List<Pose2d> wallPath, BlenderWalls WallName) {
        for (int x = 0; x < wallPath.size(); x++) {
            comparedPointX = wallPath.get(x).getX();
            comparedPointY = wallPath.get(x).getY();

            currentPointY = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getY();
            currentPointX = LimelightHelpers.getBotPose2d(Constants.SHOOTER_LIMELIGHT_NAME).getX();

            if (Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                    + Math.pow(comparedPointY - currentPointY, 2)) < howFarIsClosestPoint) {
                howFarIsClosestPoint = Math.sqrt(Math.pow(comparedPointX - currentPointX, 2)
                        + Math.pow(comparedPointY - currentPointY, 2));
                closestPoint = wallPath.get(x);
                closestWall = WallName;
            }
        }
    }
}
