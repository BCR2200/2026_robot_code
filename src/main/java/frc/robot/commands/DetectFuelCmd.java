// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.OURLimelightHelpers;
import frc.robot.drive.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectFuelCmd extends Command {
  /** Creates a new Drive. */
  CommandSwerveDrivetrain driveSubsystem;
  double[] johnJawbreakerTaylorPercentages;

  public DetectFuelCmd(CommandSwerveDrivetrain driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  /**
   * Attempts to track a piece of fuel using its detected contour.
   * Drives forward if there is a target, and turns proportionally to its angle error.
   */
  public void execute() {

    // get contour from limelight
    // has detection flag, x offset from center and y offset from center in pixels
    OURLimelightHelpers.LimelightContour contour = OURLimelightHelpers.getContour();

    // if there are no targets, don't do anything
    if (!contour.hasTarget()) {
      driveSubsystem.applyRequest(SwerveRequest.Idle::new);
    } 
    else {
      // otherwise, drive towards the contour center
      // do not rotate tiny amounts (deadzone), otherwise rotate at a speed that achieves the correct angle in 1/3s
      // the 54/160 converts the pixels to degrees (assuming 320x240 resolution and 54 degree FOV)
      double rotationalRate = -3 * Math.toRadians(ExtraMath.naiveDeadzone(contour.offsetX() - 160, 10)*54/320);

      // turn proportional to angle, drive forward
      driveSubsystem.applyRequest(() ->
              new SwerveRequest.RobotCentric()
                      .withVelocityX(1) // TODO: tune values (is 1 reasonable? does the rotation work sense?)
                      .withRotationalRate(rotationalRate) // negative for clockwise
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.applyRequest(() -> new RobotCentric().withVelocityX(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
