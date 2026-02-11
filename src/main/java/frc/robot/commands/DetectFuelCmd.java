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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.applyRequest(() -> new RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));

    johnJawbreakerTaylorPercentages = OURLimelightHelpers.getJohnJawbreakerTaylorPercentages();

    // If the largest percentage is greater than 1, we know that we have a valid target
    if (Math.max(johnJawbreakerTaylorPercentages[0],
        Math.max(johnJawbreakerTaylorPercentages[1], johnJawbreakerTaylorPercentages[2])) > 1) {

      // Left
      if (johnJawbreakerTaylorPercentages[0] > johnJawbreakerTaylorPercentages[1]
          && johnJawbreakerTaylorPercentages[0] > johnJawbreakerTaylorPercentages[2]) {
        driveSubsystem.applyRequest(() -> 
          new RobotCentric().withVelocityX(0).withRotationalRate(1)
        );
      }
      // Right
      else if (johnJawbreakerTaylorPercentages[2] > johnJawbreakerTaylorPercentages[0]
          && johnJawbreakerTaylorPercentages[2] > johnJawbreakerTaylorPercentages[1]) {
        driveSubsystem.applyRequest(() -> 
          new RobotCentric().withVelocityX(0).withRotationalRate(-1)
        );
      } 
      // Centre
      else {
        driveSubsystem.applyRequest(() -> 
          new RobotCentric().withVelocityX(0.5).withRotationalRate(0)
        );
      }
    }

  }

  // TODO: Integrate (rename to execute()) and tune

  /**
   * Attempts to track a piece of fuel using its detected contour.
   * Drives forward if there is a target, and turns proportionally to its angle error.
   */
  @SuppressWarnings("unused")
  public void executeAlt() {

    // get contour from limelight
    // has centre X/Y, detection flag
    OURLimelightHelpers.LimelightContour contour = OURLimelightHelpers.getContour();

    // if there are no targets, don't do anything
    if (!contour.hasTarget()) {
      driveSubsystem.applyRequest(SwerveRequest.Idle::new);
    } else {
      // otherwise, drive towards it
      // do not rotate tiny amounts (deadzone), otherwise rotate at a speed that achieves the correct angle in 1/3s
      double rotationalRate = -3 * Math.toRadians(ExtraMath.naiveDeadzone(contour.offsetX(), 5 /* deg */));

      // turn proportional to angle, drive forward
      driveSubsystem.applyRequest(() ->
              new SwerveRequest.RobotCentric()
                      .withVelocityX(1) // TODO: tune values (is 1 reasonable?)
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
