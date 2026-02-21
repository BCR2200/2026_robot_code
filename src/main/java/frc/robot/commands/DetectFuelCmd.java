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
  @Override
  public void execute() {

    OURLimelightHelpers.LimelightContour contour = OURLimelightHelpers.getContour();
    SmartDashboard.putData("contour", contour);

    // if there are no targets, don't do anything
    if (!contour.hasTarget()) {
      driveSubsystem.applyRequest(SwerveRequest.Idle::new);
    } 
    else {
      // otherwise, drive towards the contour center
      // do not rotate tiny amounts (deadzone of 1 degree), 
      // otherwise rotate at a speed that achieves the correct angle in 1/3s
      // 

      double rotationalRadsWithDeadzone = Math.toRadians(ExtraMath.naiveDeadzone(contour.degreesX(), 1));
      // P constant to convert from relative offset to rotations per second. 
      // 1 means... will rotate X offset rads in 1 second. 
      var radsPerSecond = rotationalRadsWithDeadzone / 1.0; 
      // var forwardSpeed = 0.15; // TODO: how fast should it drive forward?

      // TODO: Maybe add a minimum rotational rate (rads/second)

      // turn proportional to angle, drive forward
      driveSubsystem.applyRequest(() ->
              new SwerveRequest.RobotCentric()
                      // .withVelocityX(forwardSpeed) 
                      .withRotationalRate(radsPerSecond) // not negative for clockwise, but it's upside-down
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
