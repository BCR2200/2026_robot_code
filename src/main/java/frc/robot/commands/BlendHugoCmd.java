package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.RobotContainer;

/**
 * Command that drives the robot along the outside of the field, hugging the nearest wall.
 *
 * This command finds the nearest wall and uses a P loop to maintain a set distance from it,
 * while allowing the driver to control movement along the wall and rotation.
 *
 * Field dimensions (2026 FRC): 16.54m long (X axis) x 8.05m wide (Y axis)
 */
public class BlendHugoCmd extends Command {

    private final RobotContainer robot;

    // Field dimensions in meters (2026 FRC field)
    private static final double FIELD_LENGTH = 16.54; // X axis
    private static final double FIELD_WIDTH = 8.05;   // Y axis

    // Wall positions in blue field coordinates
    // In WPILib blue alliance coords: +X is away from blue driver station, +Y is to the left
    private static final double MIN_Y_WALL = 0.0;         // Right wall from blue operator's view
    private static final double MAX_Y_WALL = FIELD_WIDTH; // Left wall from blue operator's view
    private static final double MIN_X_WALL = 0.0;         // Blue alliance wall
    private static final double MAX_X_WALL = FIELD_LENGTH; // Red alliance wall

    // Target distance from wall (robot center to wall)
    private static final double WALL_OFFSET = 0.5; // meters

    // P gain for driving towards the wall
    private static final double kP = 2.0;

    // Maximum correction velocity (m/s)
    private static final double MAX_CORRECTION_VELOCITY = 3.0;

    private enum NearestWall {
        MIN_Y,  // Y = 0 (right wall from blue's view)
        MAX_Y,  // Y = FIELD_WIDTH (left wall from blue's view)
        MIN_X,  // X = 0 (blue alliance wall)
        MAX_X   // X = FIELD_LENGTH (red alliance wall)
    }

    public BlendHugoCmd(RobotContainer robot) {
        addRequirements(robot.drivetrain);
        this.robot = robot;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = robot.drivetrain.getState().Pose;
        double x = currentPose.getX();
        double y = currentPose.getY();

        // Calculate perpendicular distance to each wall (all in blue field coordinates)
        double distToMinY = y - MIN_Y_WALL;
        double distToMaxY = MAX_Y_WALL - y;
        double distToMinX = x - MIN_X_WALL;
        double distToMaxX = MAX_X_WALL - x;

        // Find the nearest wall
        double minDist = Math.min(Math.min(distToMinY, distToMaxY),
                                   Math.min(distToMinX, distToMaxX));

        NearestWall nearestWall;
        if (minDist == distToMinY) {
            nearestWall = NearestWall.MIN_Y;
        } else if (minDist == distToMaxY) {
            nearestWall = NearestWall.MAX_Y;
        } else if (minDist == distToMinX) {
            nearestWall = NearestWall.MIN_X;
        } else {
            nearestWall = NearestWall.MAX_X;
        }

        SmartDashboard.putString("WallHug/NearestWall", nearestWall.toString());
        SmartDashboard.putNumber("WallHug/DistanceToWall", minDist);

        // Calculate P loop correction in FIELD coordinates (blue alliance reference frame)
        // These will be transformed to operator perspective before applying
        double correctionFieldX = 0;
        double correctionFieldY = 0;

        // Driver input components (already in operator perspective)
        double driverVelX = -RobotContainer.driverY * RobotContainer.MaxSpeed;
        double driverVelY = -RobotContainer.driverX * RobotContainer.MaxSpeed;

        switch (nearestWall) {
            case MIN_Y:
                // Target Y = WALL_OFFSET (just inside the Y=0 wall)
                double errorMinY = y - WALL_OFFSET;
                correctionFieldY = -kP * errorMinY; // Negative to decrease Y towards the wall
                correctionFieldY = ExtraMath.clamp(correctionFieldY, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);
                SmartDashboard.putNumber("WallHug/Error", errorMinY);
                break;

            case MAX_Y:
                // Target Y = FIELD_WIDTH - WALL_OFFSET (just inside the Y=8.05 wall)
                double errorMaxY = (FIELD_WIDTH - WALL_OFFSET) - y;
                correctionFieldY = kP * errorMaxY; // Positive to increase Y towards the wall
                correctionFieldY = ExtraMath.clamp(correctionFieldY, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);
                SmartDashboard.putNumber("WallHug/Error", errorMaxY);
                break;

            case MIN_X:
                // Target X = WALL_OFFSET (just inside the X=0 wall)
                double errorMinX = x - WALL_OFFSET;
                correctionFieldX = -kP * errorMinX; // Negative to decrease X towards the wall
                correctionFieldX = ExtraMath.clamp(correctionFieldX, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);
                SmartDashboard.putNumber("WallHug/Error", errorMinX);
                break;

            case MAX_X:
            default:
                // Target X = FIELD_LENGTH - WALL_OFFSET (just inside the X=16.54 wall)
                double errorMaxX = (FIELD_LENGTH - WALL_OFFSET) - x;
                correctionFieldX = kP * errorMaxX; // Positive to increase X towards the wall
                correctionFieldX = ExtraMath.clamp(correctionFieldX, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);
                SmartDashboard.putNumber("WallHug/Error", errorMaxX);
                break;
        }

        // Transform field-coordinate correction to operator perspective
        // OperatorPerspective rotates velocities by the operator angle, so we need to
        // pre-rotate our field corrections to cancel this out and get actual field motion
        Rotation2d operatorAngle = robot.drivetrain.getOperatorForwardDirection();
        double cos = operatorAngle.getCos();
        double sin = operatorAngle.getSin();

        // Rotate correction from field coords to operator coords
        // (inverse rotation: multiply by transpose of rotation matrix)
        double correctionOperX = correctionFieldX * cos + correctionFieldY * sin;
        double correctionOperY = -correctionFieldX * sin + correctionFieldY * cos;

        // Combine driver input (already in operator space) with transformed correction
        // For Y-wall (MIN_Y/MAX_Y): driver controls X (along wall), P loop controls Y
        // For X-wall (MIN_X/MAX_X): driver controls Y (along wall), P loop controls X
        double velocityX;
        double velocityY;

        if (nearestWall == NearestWall.MIN_Y || nearestWall == NearestWall.MAX_Y) {
            // Hugging a Y-axis wall: driver controls movement along wall (X), P loop controls perpendicular (Y)
            velocityX = driverVelX;
            velocityY = correctionOperY;
        } else {
            // Hugging an X-axis wall: driver controls movement along wall (Y), P loop controls perpendicular (X)
            velocityX = correctionOperX;
            velocityY = driverVelY;
        }

        SmartDashboard.putNumber("WallHug/CorrectionFieldX", correctionFieldX);
        SmartDashboard.putNumber("WallHug/CorrectionFieldY", correctionFieldY);
        SmartDashboard.putNumber("WallHug/VelocityX", velocityX);
        SmartDashboard.putNumber("WallHug/VelocityY", velocityY);

        // Apply the control with driver rotation always available
        robot.drivetrain.setControl(robot.driveFC
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
    }

    @Override
    public void end(boolean interrupted) {
        robot.drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
