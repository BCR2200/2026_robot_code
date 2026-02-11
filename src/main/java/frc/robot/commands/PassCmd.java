// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassCmd extends Command {
  /** Creates a new Drive. */
  ShooterSubsystem shooterSubsystem;
  CommandSwerveDrivetrain driveSubsystem;

  public PassCmd(CommandSwerveDrivetrain driveSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, driveSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.applyRequest(() -> new FieldCentricFacingAngle().withTargetDirection(new Rotation2d(Math.PI)));

    double distance;
    if (Robot.alliance == Alliance.Blue) {
      distance = LimelightHelpers.getBotPose2d_wpiBlue(Constants.shooterLimelightName).getX();
    }
    else {
      distance = LimelightHelpers.getBotPose2d_wpiRed(Constants.shooterLimelightName).getX();
    }
    if (distance > 5 && distance < 15) { // TODO GET VALUES
      shooterSubsystem.setActuatorToPassPosition(distance);
    }
    new ShootCmd(shooterSubsystem);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
