// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.OURLimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectFuelCmd extends Command {
  /** Creates a new Drive. */
  private RobotContainer robotContainer;
  public static double radsPerSecond;

  public DetectFuelCmd(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotContainer = robotContainer;
  }

  /**
   * Attempts to track a piece of fuel using its detected contour.
   * Drives forward if there is a target, and turns proportionally to its angle error.
   */
  @Override
  public void execute() {

    OURLimelightHelpers.LimelightContour contour = OURLimelightHelpers.getContour();
    SmartDashboard.putData("contour", contour);

    // if there are no targets, don't do anything
    if (!contour.hasTarget()) {
      robotContainer.fuelTracking = false;
    } 
    else {
      // otherwise, drive towards the contour center
      // do not rotate tiny amounts (deadzone of 1 degree), 

      double rotationalRadsWithDeadzone = Math.toRadians(ExtraMath.naiveDeadzone(contour.degreesX(), 1));
      radsPerSecond = rotationalRadsWithDeadzone / 0.5; // takes half a second to rotate to target, TODO tune 
      robotContainer.fuelTracking = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotContainer.fuelTracking = false;
  }
}
