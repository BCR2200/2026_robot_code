package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class DriveToOutpostCmd extends Command {

    /**
     * P value (proportional output) used in driveToPose.
     */
    @NotLogged
    public static final double TRANSLATION_P = 6.0;

    public Pose2d finalTarget;
    public Pose2d initialTarget;
    public boolean goneToInitialPos = false;

    private final CommandSwerveDrivetrain drivetrain;
    private final RobotContainer robotContainer;

    public DriveToOutpostCmd(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        addRequirements(drivetrain);
    }

    public double getXToTarget(Pose2d targetPose) {
        Pose2d robotPose2d = drivetrain.getState().Pose;
        return targetPose.getX() - robotPose2d.getX();
    }

    public double getYToTarget(Pose2d targetPose) {
        Pose2d robotPose2d = drivetrain.getState().Pose;
        return targetPose.getY() - robotPose2d.getY();
    }

    /**
     * Returns true if the robot is at the specified postion, false otherwise
     * @param targetPos the target pos
     * @param threashold the threashold that the robot can be within to count as there
     * @return if robot is at the targetPos
     */
    public boolean atTargetPos(Pose2d targetPos, double threashold) {
        return getDistanceToTarget(targetPos) < threashold;
    }

    public double getDistanceToTarget(Pose2d targetPose) {
        Pose2d robotPose = drivetrain.getState().Pose;
        return robotPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    private SwerveRequest.FieldCentricFacingAngle driveToPose(Pose2d target, double maxSpeed) {
        return robotContainer.driveFCFAVelocityMode.withVelocityX(ExtraMath.clampedDeadzone(getXToTarget(target)*TRANSLATION_P, maxSpeed, 0.0001))
                .withVelocityY(ExtraMath.clampedDeadzone(getYToTarget(target)*TRANSLATION_P, maxSpeed, 0.0001))
                .withTargetDirection(target.getRotation())
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    }

    @Override
    public void initialize() {
        goneToInitialPos = false;
        if (Robot.alliance == DriverStation.Alliance.Red) {
            initialTarget = Constants.OUTPOST_RED_INITIAL;
            finalTarget = Constants.OUTPOST_RED_FINAL;
        }
        else {
            initialTarget = Constants.OUTPOST_BLUE_INITIAL;
            finalTarget = Constants.OUTPOST_BLUE_FINAL;
        }
    }

    @Override
    public void execute() {
        if (atTargetPos(finalTarget, 0.015)) { // At final
            drivetrain.setControl(robotContainer.driveFC.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        } 
        else if (atTargetPos(initialTarget, 0.06) || goneToInitialPos) { // Past initial
            goneToInitialPos = true;
            drivetrain.setControl(driveToPose(finalTarget, 0.6));
        } 
        else { // Not at initial
            drivetrain.setControl(driveToPose(initialTarget, 2));
        }
    }

    @Override
    public void end(boolean interrupted) {
        goneToInitialPos = false;
    }
}
