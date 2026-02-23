package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAtHub extends Command {
  RobotContainer rc;
  ShooterSubsystem johnShooterSubsystem;
  ShooterSubsystem jawbreakerShooterSubsystem;
  ShooterSubsystem taylorShooterSubsystem;

  public ShootAtHub (RobotContainer rc){
    this.rc = rc;
    this.johnShooterSubsystem = rc.shooterSubsystemJohn;
    this.jawbreakerShooterSubsystem = rc.shooterSubsystemJawbreaker;
    this.taylorShooterSubsystem = rc.shooterSubsystemTaylor;
  }

  @Override
  public void initialize() {
    rc.shootingAtHub = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
