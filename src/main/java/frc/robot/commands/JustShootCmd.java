package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    johnShooterSubsystem.setIsShooting(true);
    jawbreakerShooterSubsystem.setIsShooting(true);
    taylorShooterSubsystem.setIsShooting(true);
    if (johnShooterSubsystem.isShooterAtSpeed()) {
      johnShooterSubsystem.setIsFeeding(true);
      jawbreakerShooterSubsystem.setIsFeeding(true);
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
