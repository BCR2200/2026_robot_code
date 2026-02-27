package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAt extends Command {
  RobotContainer rc;
  ShooterSubsystem johnShooterSubsystem;
  ShooterSubsystem jawbreakerShooterSubsystem;
  ShooterSubsystem taylorShooterSubsystem;

  public ShootAt (RobotContainer rc){
    this.rc = rc;
    this.johnShooterSubsystem = rc.shooterSubsystemJohn;
    this.jawbreakerShooterSubsystem = rc.shooterSubsystemJawbreaker;
    this.taylorShooterSubsystem = rc.shooterSubsystemTaylor;
    addRequirements(johnShooterSubsystem, jawbreakerShooterSubsystem, taylorShooterSubsystem);
  }

  @Override
  public void initialize() {
    rc.shootingAtHub = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Only allow passing when in the neutral zone
    double distance = rc.drivetrain.getState().Pose.getX();

    if (distance > 4.625 && distance < 11.916) { // Neutral zone
      rc.passing = true;
      johnShooterSubsystem.setActuatorToPassPosition(distance);
      jawbreakerShooterSubsystem.setActuatorToPassPosition(distance);
      taylorShooterSubsystem.setActuatorToPassPosition(distance);
    }
    else {
      rc.passing = false;
    }

    activateShooters();
    
  }

  private void activateShooters() {
    johnShooterSubsystem.setIsShooting(true);
    jawbreakerShooterSubsystem.setIsShooting(true);
    taylorShooterSubsystem.setIsShooting(true);

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
