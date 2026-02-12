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
import frc.robot.subsystems.FloorFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassCmd extends Command {
  /** Creates a new Drive. */
  ShooterSubsystem shooterSubsystem1;
  ShooterSubsystem shooterSubsystem2;
  ShooterSubsystem shooterSubsystem3;
  CommandSwerveDrivetrain driveSubsystem;
  FloorFeedSubsystem floorFeedSubsystem;

  public PassCmd(CommandSwerveDrivetrain driveSubsystem,
                 ShooterSubsystem shooterSubsystem1,
                 ShooterSubsystem shooterSubsystem2,
                 ShooterSubsystem shooterSubsystem3,
                 FloorFeedSubsystem floorFeedSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem1, shooterSubsystem2, shooterSubsystem3, driveSubsystem, floorFeedSubsystem);
    this.shooterSubsystem1 = shooterSubsystem1;
    this.shooterSubsystem2 = shooterSubsystem2;
    this.shooterSubsystem3 = shooterSubsystem3;
    this.driveSubsystem = driveSubsystem;
    this.floorFeedSubsystem = floorFeedSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.applyRequest(() -> new FieldCentricFacingAngle().withTargetDirection(new Rotation2d(Math.PI)));

    // Only allow passing when in the neutral zone
    double distance;
    if (Robot.alliance == Alliance.Blue) {
      distance = LimelightHelpers.getBotPose2d_wpiBlue(Constants.SHOOTER_LIMELIGHT_NAME).getX();
    }
    else {
      distance = LimelightHelpers.getBotPose2d_wpiRed(Constants.SHOOTER_LIMELIGHT_NAME).getX();
    }
    if (distance > 4.625 && distance < 11.916) { // Neutral zone
      shooterSubsystem1.setActuatorToPassPosition(distance);
      shooterSubsystem2.setActuatorToPassPosition(distance);
      shooterSubsystem3.setActuatorToPassPosition(distance);
    }

    shooterSubsystem1.setIsShooting(true);
    shooterSubsystem2.setIsShooting(true);
    shooterSubsystem3.setIsShooting(true);
    if (shooterSubsystem1.isShooterAtSpeed()) {
      shooterSubsystem1.setIsFeeding(true);
      shooterSubsystem2.setIsFeeding(true);
      shooterSubsystem3.setIsFeeding(true);
      floorFeedSubsystem.setIsFeeding(true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem1.setIsShooting(false);
    shooterSubsystem1.setIsFeeding(false);
    shooterSubsystem2.setIsShooting(false);
    shooterSubsystem2.setIsFeeding(false);
    shooterSubsystem3.setIsShooting(false);
    shooterSubsystem3.setIsFeeding(false);
    floorFeedSubsystem.setIsFeeding(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
