// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final MTFTFFTHTTSSubsystem m_feederSubsystem = new MTFTFFTHTTSSubsystem();
  private final LinearActuator m_linearActuator = new LinearActuator(Constants.LINEAR_ACTUATOR_CHANNEL, 100, 20);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Shooter Speed Controls
    m_driverController.a().onTrue(new InstantCommand(() -> m_shooterSubsystem.incrementShooterSpeed()));
    m_driverController.b().onTrue(new InstantCommand(() -> m_shooterSubsystem.decrementShooterSpeed()));
    // Shooter On/off Controls
    m_driverController.rightTrigger().whileTrue(new InstantCommand(() -> m_shooterSubsystem.setIsShooting(true)))
                                      .whileFalse(new InstantCommand(() -> m_shooterSubsystem.setIsShooting(false)));

    // Feeder Speed Controls
    m_driverController.y().onTrue(new InstantCommand(() -> m_feederSubsystem.incrementFeedingSpeed()));
    m_driverController.x().onTrue(new InstantCommand(() -> m_feederSubsystem.decrementFeedingSpeed()));
    // Feeder On/off Controls
    m_driverController.leftTrigger().whileTrue(new InstantCommand(() -> m_feederSubsystem.setIsFeeding(true)))
                                     .whileFalse(new InstantCommand(() -> m_feederSubsystem.setIsFeeding(false)));

    // Linear Actuator Controls, 0-100 mm
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_linearActuator.incrementPosition(-5)));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_linearActuator.incrementPosition(5)));

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
