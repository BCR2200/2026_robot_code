package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimbSubsystem;

import static edu.wpi.first.units.Units.*;

public class ClimbCommand extends Command {

    /**
     * P value (proportional output) used in driveToPose.
     */
    @NotLogged
    public static final double TRANSLATION_P = 3.0;

    public static final Pose2d BLUE_L_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(1.535, Meters),
            Distance.ofBaseUnits(4.155, Meters),
            Rotation2d.fromDegrees(180)
    );
    public static final Pose2d BLUE_L_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(1.355, Meters),
            Distance.ofBaseUnits(4.155, Meters),
            Rotation2d.fromDegrees(180)
    );
    public static final Pose2d BLUE_R_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(1.535, Meters),
            Distance.ofBaseUnits(3.290, Meters),
            Rotation2d.fromDegrees(180)
    );
    public static final Pose2d BLUE_R_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(1.355, Meters),
            Distance.ofBaseUnits(3.290, Meters),
            Rotation2d.fromDegrees(180)
    );
    public static final Pose2d RED_L_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(15.0, Meters),
            Distance.ofBaseUnits(3.89, Meters),
            Rotation2d.kZero
    );
    public static final Pose2d RED_L_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(15.20, Meters),
            Distance.ofBaseUnits(3.89, Meters),
            Rotation2d.kZero
    );
    public static final Pose2d RED_R_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(15.0, Meters),
            Distance.ofBaseUnits(4.73, Meters),
            Rotation2d.kZero
    );
    public static final Pose2d RED_R_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(15.22, Meters),
            Distance.ofBaseUnits(4.73, Meters),
            Rotation2d.kZero
    );

    public Pose2d targetClimbInitial = RED_R_CLIMB_INITIAL;
    public Pose2d targetClimbFinal = RED_R_CLIMB_FINAL;

    private final CommandSwerveDrivetrain drivetrain;
    private final ClimbSubsystem climberSubsystem;
    private final RobotContainer robot;
    private final boolean isOnRight;
    private boolean goneToInitialPos = false;

    public ClimbCommand(RobotContainer robot, boolean isRight) {
        this.robot = robot;
        this.drivetrain = robot.drivetrain;
        this.climberSubsystem = robot.climberSubsystem;
        this.isOnRight = isRight;
        addRequirements(drivetrain, climberSubsystem);
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

    private SwerveRequest.FieldCentricFacingAngle driveToPose(Pose2d target) {
        return robot.driveFCFAVelocityMode.withVelocityX(ExtraMath.clampedDeadzone(getXToTarget(target)*-TRANSLATION_P, 1, 0.03))
                .withVelocityY(ExtraMath.clampedDeadzone(getYToTarget(target)*-TRANSLATION_P, 1, 0.03))
                .withTargetDirection(target.getRotation());
    }

    @Override
    public void initialize() {
        goneToInitialPos = false;
        if (this.isOnRight) {
            this.setupClimbR();
        } else {
            this.setupClimbL();
        }
    }

    @Override
    public void execute() {
        if (atTargetPos(targetClimbFinal, 0.03)) { // At final
            climberSubsystem.climb();
            drivetrain.setControl(robot.driveFC.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        } else if (atTargetPos(targetClimbInitial, 0.06) || goneToInitialPos) { // Past initial
            goneToInitialPos = true;
            drivetrain.setControl(driveToPose(targetClimbFinal));
        } else { // Not at initial
            drivetrain.setControl(driveToPose(targetClimbInitial));
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.goHome();
    }

    public ClimbCommand setupClimbL() {
        if (Robot.alliance == DriverStation.Alliance.Red) {
            targetClimbFinal = RED_L_CLIMB_FINAL;
            targetClimbInitial = RED_L_CLIMB_INITIAL;
        } else {
            targetClimbFinal = BLUE_L_CLIMB_FINAL;
            targetClimbInitial = BLUE_L_CLIMB_INITIAL;
        }
        return this;
    }
    public ClimbCommand setupClimbR() {
        if (Robot.alliance == DriverStation.Alliance.Red) {
            targetClimbFinal = RED_R_CLIMB_FINAL;
            targetClimbInitial = RED_R_CLIMB_INITIAL;
        } else {
            targetClimbFinal = BLUE_R_CLIMB_FINAL;
            targetClimbInitial = BLUE_R_CLIMB_INITIAL;
        }
        return this;
    }
}
