// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.OURLimelightHelpers;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectFuelCmd extends Command {
  /** Creates a new Drive. */
  private RobotContainer robotContainer;
  @Logged
  private OURLimelightHelpers.LimelightContour contour;

  public DetectFuelCmd(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotContainer = robotContainer;
  }

  /**
   * Attempts to track a piece of fuel using its detected contour.
   * Drives forward if there is a target, and turns proportionally to its angle
   * error.
   */
  @Override
  public void execute() {

    contour = OURLimelightHelpers.getContour();

    // if there are no targets, don't do anything
    if (!contour.hasTarget()) {
      robotContainer.drivetrain.setControl(
          robotContainer.driveFC
              .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
              .withVelocityX(-RobotContainer.driverY * RobotContainer.MaxSpeed)
              .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed)
              .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
      );
    } else {
      // otherwise, drive towards the contour center
      // do not rotate tiny amounts (deadzone of 1 degree),
      // TODO determine velocity x and y

      double rotationalRadsWithDeadzone = Math.toRadians(ExtraMath.naiveDeadzone(contour.degreesX(), 0));
      double radsPerSecond = rotationalRadsWithDeadzone / 0.5; // takes half a second to rotate to target, TODO tune
      robotContainer.drivetrain.setControl(
          robotContainer.driveRC
              .withRotationalRate(-radsPerSecond)
              .withVelocityX(0.5 * RobotContainer.MaxSpeed)
              .withVelocityY(0)
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // nothing to do
  }
}
