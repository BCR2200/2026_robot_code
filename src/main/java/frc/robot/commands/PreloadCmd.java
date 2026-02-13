// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PreloadCmd extends Command {
  /** Creates a new Drive. */
  ShooterSubsystem shooterSubsystem1;
  ShooterSubsystem shooterSubsystem2;
  ShooterSubsystem shooterSubsystem3;

  public PreloadCmd(ShooterSubsystem shooterSubsystem1, ShooterSubsystem shooterSubsystem2, ShooterSubsystem shooterSubsystem3) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem1, shooterSubsystem2, shooterSubsystem3);
    this.shooterSubsystem1 = shooterSubsystem1;
    this.shooterSubsystem2 = shooterSubsystem2;
    this.shooterSubsystem3 = shooterSubsystem3;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem1.setIsPreloading(true);
    shooterSubsystem2.setIsPreloading(true);
    shooterSubsystem3.setIsPreloading(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem1.setIsPreloading(false);
    shooterSubsystem2.setIsPreloading(false);
    shooterSubsystem3.setIsPreloading(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
