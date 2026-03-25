package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

import static edu.wpi.first.units.Units.*;

public class DriveAlongDriverWallCmd extends Command {

    /**
     * P value (proportional output) used in driveToPose.
     */
    @NotLogged
    public static final double TRANSLATION_P = 6.0;

    public Pose2d initialTarget;
    public Pose2d middleTarget;
    public Pose2d finalTarget;

    public boolean goneToInitialPos = false;
    public boolean goneToMiddlePos = false;

    public static final Pose2d RED_RIGHT_INITIAL = Constants.OUTPOST_RED_INITIAL;
    public static final Pose2d RED_RIGHT_FINAL = new Pose2d(
        Distance.ofBaseUnits(Constants.OUTPOST_RED_FINAL.getX() + 0.25, Meters),
        Distance.ofBaseUnits(Constants.OUTPOST_RED_FINAL.getY() - 0.5, Meters),
        Rotation2d.kCW_90deg
    );
    public static final Pose2d RED_LEFT_INITIAL = new Pose2d(
        Distance.ofBaseUnits(Constants.OUTPOST_RED_INITIAL.getX(), Meters),
        Distance.ofBaseUnits(Constants.OUTPOST_BLUE_INITIAL.getY(), Meters),
        Rotation2d.kZero
    );
    public static final Pose2d RED_LEFT_FINAL = new Pose2d(
        Distance.ofBaseUnits(Constants.OUTPOST_RED_INITIAL.getX() + 0.25, Meters),
        Distance.ofBaseUnits(Constants.OUTPOST_BLUE_INITIAL.getY() + 0.5, Meters),
        Rotation2d.kCCW_90deg
    );

    public static final Pose2d BLUE_RIGHT_INITIAL = Constants.OUTPOST_BLUE_INITIAL;
    public static final Pose2d BLUE_RIGHT_FINAL = new Pose2d(
        Distance.ofBaseUnits(Constants.OUTPOST_BLUE_FINAL.getX() - 0.25, Meters),
        Distance.ofBaseUnits(Constants.OUTPOST_BLUE_FINAL.getY() + 0.5, Meters),
        Rotation2d.kCCW_90deg
    );
    public static final Pose2d BLUE_LEFT_INITIAL = new Pose2d(
        Distance.ofBaseUnits(Constants.OUTPOST_BLUE_INITIAL.getX(), Meters),
        Distance.ofBaseUnits(Constants.OUTPOST_RED_INITIAL.getY(), Meters),
        Rotation2d.k180deg
    );
    public static final Pose2d BLUE_LEFT_FINAL = new Pose2d(
        Distance.ofBaseUnits(Constants.OUTPOST_BLUE_INITIAL.getX() - 0.25, Meters),
        Distance.ofBaseUnits(Constants.OUTPOST_RED_INITIAL.getY() - 0.5, Meters),
        Rotation2d.kCW_90deg
    );

    private final CommandSwerveDrivetrain drivetrain;
    private final RobotContainer robotContainer;

    public DriveAlongDriverWallCmd(RobotContainer robotContainer) {
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

    /**
     * @param target
     * @param maxSpeed in m/s
     * @param maxAbsRotationalRate in radians/s
     * @return
     */
    private SwerveRequest.FieldCentricFacingAngle driveToPose(Pose2d target, double maxSpeed, double maxAbsRotationalRate, boolean rotate) {
        return robotContainer.driveFCFAVelocityMode.withVelocityX(ExtraMath.clampedDeadzone(getXToTarget(target)*TRANSLATION_P, maxSpeed, 0.0001))
                .withVelocityY(ExtraMath.clampedDeadzone(getYToTarget(target)*TRANSLATION_P, maxSpeed, 0.0001))
                .withTargetDirection(rotate ? target.getRotation() : drivetrain.getState().Pose.getRotation())
                .withMaxAbsRotationalRate(maxAbsRotationalRate)
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    }

    @Override
    public void initialize() {
        goneToInitialPos = false;
        goneToMiddlePos = false;

        if (Robot.alliance == DriverStation.Alliance.Red) {
            if (drivetrain.getState().Pose.getY() < 4) {
                initialTarget = RED_LEFT_INITIAL;
                middleTarget = RED_LEFT_FINAL;
                finalTarget = RED_RIGHT_FINAL;
            }
            else {
                initialTarget = RED_RIGHT_INITIAL;
                middleTarget = RED_RIGHT_FINAL;
                finalTarget = RED_LEFT_FINAL;
            }
        }
        else {
            if (drivetrain.getState().Pose.getY() > 4) {
                initialTarget = BLUE_LEFT_INITIAL;
                middleTarget = BLUE_LEFT_FINAL;
                finalTarget = BLUE_RIGHT_FINAL;
            }
            else {
                initialTarget = BLUE_RIGHT_INITIAL;
                middleTarget = BLUE_RIGHT_FINAL;
                finalTarget = BLUE_LEFT_FINAL;
            }
        }
        robotContainer.intakeSubsystem.setIsIntaking(true);
        robotContainer.intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMaxExtensionPos);
    }

    @Override
    public void execute() {
        if (atTargetPos(finalTarget, 0.015)) { // At final
            drivetrain.setControl(robotContainer.driveFC.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        }
        else if (atTargetPos(middleTarget, 0.06) || goneToMiddlePos) { // Past middle
            goneToMiddlePos = true;
            drivetrain.setControl(driveToPose(finalTarget, 0.6, 0.001, false));
        }
        else if (atTargetPos(initialTarget, 0.06) || goneToInitialPos) { // Past initial
            goneToInitialPos = true;
            drivetrain.setControl(driveToPose(middleTarget, 0.6, 0.5, true));
        }
        else { // Not at initial
            drivetrain.setControl(driveToPose(initialTarget, 2, 0, true)); // maxRotationalRate of 0 means no max
        }
    }

    @Override
    public void end(boolean interrupted) {
        goneToInitialPos = false;
        goneToMiddlePos = false;
        robotContainer.intakeSubsystem.setIsIntaking(false);
    }
}
