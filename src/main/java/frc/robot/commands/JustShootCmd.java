package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class JustShootCmd extends Command{
  ShooterSubsystem johnShooterSubsystem;
  ShooterSubsystem jawbreakerShooterSubsystem;
  ShooterSubsystem taylorShooterSubsystem;

  public JustShootCmd (ShooterSubsystem johnShooterSubsystem, ShooterSubsystem jawbreakerShooterSubsystem, ShooterSubsystem taylorShooterSubsystem){
    this.johnShooterSubsystem = johnShooterSubsystem;
    this.jawbreakerShooterSubsystem = jawbreakerShooterSubsystem;
    this.taylorShooterSubsystem = taylorShooterSubsystem;
  }

  @Override
  public void initialize() {
  }

  public static Command getStartCommand(ShooterSubsystem johnShooterSubsystem, ShooterSubsystem jawbreakerShooterSubsystem, ShooterSubsystem taylorShooterSubsystem) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        johnShooterSubsystem.setIsShooting(true);
        jawbreakerShooterSubsystem.setIsShooting(true);
        taylorShooterSubsystem.setIsShooting(true);
      }),
      new WaitCommand(1.0),
      new InstantCommand(() -> {
        johnShooterSubsystem.setIsFeeding(true);
        jawbreakerShooterSubsystem.setIsFeeding(true);
        taylorShooterSubsystem.setIsFeeding(true);
      })
    );
  }
  public static Command getStopCommand(ShooterSubsystem johnShooterSubsystem, ShooterSubsystem jawbreakerShooterSubsystem, ShooterSubsystem taylorShooterSubsystem) {
    return new InstantCommand(() -> {
      johnShooterSubsystem.setIsShooting(false);
      jawbreakerShooterSubsystem.setIsShooting(false);
      taylorShooterSubsystem.setIsShooting(false);
      johnShooterSubsystem.setIsFeeding(false);
      jawbreakerShooterSubsystem.setIsFeeding(false);
      taylorShooterSubsystem.setIsFeeding(false);
    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    johnShooterSubsystem.setIsShooting(true);
    jawbreakerShooterSubsystem.setIsShooting(true);
    taylorShooterSubsystem.setIsShooting(true);

    if (johnShooterSubsystem.isShooterAtSpeed()) {
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
