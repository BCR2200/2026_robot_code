package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class BlendAdamModeCmd extends Command {

    private final RobotContainer robot;

    private CommandSwerveDrivetrain drivetrain;
    private Pose2d currentPose2d;

    // correct these values for the errors on the blue side, and for a rotation buffer

    // in meters
    private final double blueHUBLine = 5.7;
    private final double redHUBLine = 11.6;

    // FIND VALUES
    private final double REDDRIVERLINE = 0;
    private final double BLUEDRIVERLINE = 0;

    // left/right from blue's perspective
    private double wallLeftLine = 7.590;
    private double wallRightLine = 0.319;

    public BlendAdamModeCmd(RobotContainer robot) {
        addRequirements(robot.drivetrain);
        this.drivetrain = robot.drivetrain;
        this.robot = robot;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        /*
         * If (in a corner, allow driver X and Y)
         * If (On one of the X lines, only allow Y)
         * If (On one of the Y lines, only allow X)
         * If (Not on either, free reign)
         */

        currentPose2d = drivetrain.getState().Pose;
        var x = currentPose2d.getX();
        var y = currentPose2d.getY();

        double epsilon = 0.1;

        // Red/blue lines are parallel to Y, compare to X
        // Side lines are parallel to X, compare to Y
        // wallSomethingLine is from blue's perspective
        boolean onRedHubLine = ExtraMath.within(x, redHUBLine, epsilon);
        boolean onBlueHubLine = ExtraMath.within(x, blueHUBLine, epsilon);
        boolean onWallLFromBlueLine = ExtraMath.within(y, wallLeftLine, epsilon);
        boolean onWallRFromBlueLine = ExtraMath.within(y, wallRightLine, epsilon);

        // Driver line
        boolean onRedDriveLine = ExtraMath.within(x, REDDRIVERLINE, epsilon);
        boolean onBlueDriveLine = ExtraMath.within(x, BLUEDRIVERLINE, epsilon);

        if ((onRedHubLine || onBlueHubLine) && (onWallLFromBlueLine || onWallRFromBlueLine)) {

            // we're in a corner
            drivetrain.setControl(robot.driveFC
                    .withVelocityX(-RobotContainer.driverY * RobotContainer.MaxSpeed)
                    .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed)
                    .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
            SmartDashboard.putString("BlendState", "On Corner");

        } else if (onBlueHubLine || onRedHubLine) {

            // on a hub line (parallel to Y)
            robot.drivetrain.setControl(robot.driveFC
                    .withVelocityX(0)
                    .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed)
                    .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            );

            if (onRedHubLine) {
                SmartDashboard.putString("BlendState", "On Red Hub Line");
            } else if (onBlueHubLine) {
                SmartDashboard.putString("BlendState", "On Blue Hub Line");
            }

        } else if (onWallLFromBlueLine || onWallRFromBlueLine) {
        
            // on a left/right wall line (parallel to X)
            robot.drivetrain.setControl(robot.driveFC
                    .withVelocityX(-RobotContainer.driverY * RobotContainer.MaxSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            );

            if (onWallRFromBlueLine) {
                SmartDashboard.putString("BlendState", "On Right From Blue Line");
            } else if (onWallLFromBlueLine) {
                SmartDashboard.putString("BlendState", "On Left From Blue Line");
            }

        } else if (onBlueDriveLine || onRedDriveLine) {

            // on a Drive line (parallel to Y)
            robot.drivetrain.setControl(robot.driveFC
                    .withVelocityX(0)
                    .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed)
                    .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            );

            if (onRedDriveLine) {
                SmartDashboard.putString("BlendState", "On Red Drive Line");
            } else if (onBlueDriveLine) {
                SmartDashboard.putString("BlendState", "On Blue Drive Line");
            }
        }else {

            // somewhere in the middle or in an alliance
            SmartDashboard.putString("BlendState", "Free Roam");
            drivetrain.setControl(robot.driveFC
                    .withVelocityX(-RobotContainer.driverY * RobotContainer.MaxSpeed)
                    .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed)
                    .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            );

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
}
