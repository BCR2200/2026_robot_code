package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAt extends Command {
  RobotContainer rc;
  ShooterSubsystem shooterSubsystem;
  Timer timer;

  public ShootAt (RobotContainer rc){
    this.rc = rc;
    this.shooterSubsystem = rc.shooterSubsystem;
    addRequirements(shooterSubsystem, rc.drivetrain);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!rc.fixedPassingShot && !rc.fixedShotFromClimber && !rc.fixedShotFromHub && !rc.shooterSubsystem.isManualMode) {
      if (rc.isOutsideAllianceZone()) { // We want to pass
        rc.passing = true;
        rc.shootingAtHub = false;
      }
      else {
        rc.shootingAtHub = true;
        rc.passing = false;
      }
      rc.updateDriverInputs();
      Translation2d blueSpaceDriverInputs = new Translation2d(RobotContainer.driverX, RobotContainer.driverY)
          .rotateBy(Robot.alliance == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);

      if (rc.shootingAtHub) {

        // Check that:
        //  - we are not moving more than 10 cm/s in x/y;
        //  - the driver is not moving the left stick (more than 10%);
        //  - the robot is within 2deg of aiming at the goal
        if (ExtraMath.within(rc.drivetrain.getState().Speeds.vxMetersPerSecond, 0, 0.1) && 
            ExtraMath.within(rc.drivetrain.getState().Speeds.vyMetersPerSecond, 0, 0.1) && 
            ExtraMath.within(RobotContainer.driverX, 0, 0.1) && 
            ExtraMath.within(RobotContainer.driverY, 0, 0.1) &&
            ExtraMath.within(
              rc.drivetrain.getState().Pose.getRotation().minus(
                Rotation2d.fromDegrees(rc.getDegreesToTarget(rc.targetHub))).getDegrees(),
            0, 2)
        ) {
          rc.drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        } else {
          rc.drivetrain.setControl(rc.driveFCFA
              .withTargetDirection(Rotation2d.fromDegrees(rc.getDegreesToTarget(rc.compensatedTargetHub)).rotateBy(Rotation2d.k180deg))
              .withVelocityX(-blueSpaceDriverInputs.getY() * RobotContainer.MaxSpeed)
              .withVelocityY(-blueSpaceDriverInputs.getX() * RobotContainer.MaxSpeed)
              .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)); // Drive left with negative X
        }

      }
      else if (rc.passing) {
        rc.drivetrain.setControl(rc.driveFCFA
            .withTargetDirection(Rotation2d.fromDegrees(rc.getDegreesToTarget(rc.passTarget)).rotateBy(Rotation2d.k180deg))
            .withVelocityX(-blueSpaceDriverInputs.getY() * RobotContainer.MaxSpeed)
            .withVelocityY(-blueSpaceDriverInputs.getX() * RobotContainer.MaxSpeed)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance));
      }
    }

    activateShooters();
    
  }

  private void activateShooters() {
    shooterSubsystem.setIsShooting(true);
    if (shooterSubsystem.isShooterAtSpeed()) {
      shooterSubsystem.setIsFeeding(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.shootingAtHub = false;
    rc.passing = false;
    shooterSubsystem.setIsShooting(false);
    shooterSubsystem.setIsFeeding(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
