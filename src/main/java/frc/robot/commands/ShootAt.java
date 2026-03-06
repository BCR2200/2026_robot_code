package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAt extends Command {
  RobotContainer rc;
  ShooterSubsystem johnShooterSubsystem;
  ShooterSubsystem jawbreakerShooterSubsystem;
  ShooterSubsystem taylorShooterSubsystem;
  Timer timer;

  public ShootAt (RobotContainer rc){
    this.rc = rc;
    this.johnShooterSubsystem = rc.shooterSubsystemJohn;
    this.jawbreakerShooterSubsystem = rc.shooterSubsystemJawbreaker;
    this.taylorShooterSubsystem = rc.shooterSubsystemTaylor;
    addRequirements(johnShooterSubsystem, jawbreakerShooterSubsystem, taylorShooterSubsystem, rc.drivetrain);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (rc.isInNeutralZone()) { // Neutral zone
      rc.passing = true;
      rc.shootingAtHub = false;
      double distance = rc.drivetrain.getState().Pose.getX();

      johnShooterSubsystem.setActuatorToPassPosition(distance);
      jawbreakerShooterSubsystem.setActuatorToPassPosition(distance);
      taylorShooterSubsystem.setActuatorToPassPosition(distance);
    }
    else {
      rc.shootingAtHub = true;
      rc.passing = false;
    }
    rc.updateDriverInputs();
    if (rc.shootingAtHub) {
      rc.drivetrain.setControl(rc.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(rc.getDegreesToTarget(rc.compensatedTargetHub)))
        .withVelocityX(-RobotContainer.driverY * RobotContainer.MaxSpeed) // Drive forward with negative Y
        .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed)); // Drive left with negative X
    }
    else if (rc.passing) {
      rc.drivetrain.setControl(rc.driveFCFA
          .withTargetDirection(Rotation2d.fromDegrees(rc.getDegreesToTarget(rc.passTarget)))
          .withVelocityX(-RobotContainer.driverY * RobotContainer.MaxSpeed)
          .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed));
    }

    activateShooters();
    
  }

  private void activateShooters() {
    johnShooterSubsystem.setIsShooting(true);
    if (timer.hasElapsed(0.2)) {
      jawbreakerShooterSubsystem.setIsShooting(true);
    }
    if (timer.hasElapsed(0.4)) {
      taylorShooterSubsystem.setIsShooting(true);
    }

    if (johnShooterSubsystem.isShooterAtSpeed()) {
      // TODO: should we stop feeding later if no longer at speed?
      // Maybe... but each ball will probably reduce the shooter speed enough to stop the feeder.
      // Maybe it should only stop if it is not at speed for X time.
      johnShooterSubsystem.setIsFeeding(true);
    }
    if (jawbreakerShooterSubsystem.isShooterAtSpeed()) {
      jawbreakerShooterSubsystem.setIsFeeding(true);
    }
    if (taylorShooterSubsystem.isShooterAtSpeed()) {
      taylorShooterSubsystem.setIsFeeding(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.shootingAtHub = false;
    rc.passing = false;
    johnShooterSubsystem.setIsShooting(false);
    jawbreakerShooterSubsystem.setIsFeeding(false);
    taylorShooterSubsystem.setIsShooting(false);
    johnShooterSubsystem.setIsFeeding(false);
    jawbreakerShooterSubsystem.setIsShooting(false);
    taylorShooterSubsystem.setIsFeeding(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
