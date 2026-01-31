// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
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
  
  private static final double ACTUATOR_STEP = 0.05;

  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem m_shooterSubsystemJohn = new ShooterSubsystem( "John",
    Constants.JOHN_SHOOTER_MOTOR_ID, Constants.JOHN_FEEDER_MOTOR_ID, Constants.JOHN_LINEAR_ACTUATOR_CHANNEL,
    new Interpolator(
      new double[] {2, 4, 10, 20}, 
      new double[] {0.5, 3, 10, 70}
    ),
    new Interpolator(
      new double[] {2, 4, 10, 20}, 
      new double[] {30, 50, 80, 110}  
    )
  );
  private final ShooterSubsystem m_shooterSubsystemJawbreaker = new ShooterSubsystem( "Jawbreaker",
    Constants.JAWBREAKER_SHOOTER_MOTOR_ID, Constants.JAWBREAKER_FEEDER_MOTOR_ID, Constants.JAWBREAKER_LINEAR_ACTUATOR_CHANNEL,
    new Interpolator(
      new double[] {2, 4, 10, 20}, 
      new double[] {0.5, 3, 10, 70}
    ),
    new Interpolator(
      new double[] {2, 4, 10, 20}, 
      new double[] {30, 50, 80, 110}  
    )
  );
  private final ShooterSubsystem m_shooterSubsystemTaylor = new ShooterSubsystem( "Taylor",
    Constants.TAYLOR_SHOOTER_MOTOR_ID, Constants.TAYLOR_FEEDER_MOTOR_ID, Constants.TAYLOR_LINEAR_ACTUATOR_CHANNEL,
    new Interpolator(
      new double[] {2, 4, 10, 20}, 
      new double[] {0.5, 3, 10, 70}
    ),
    new Interpolator(
      new double[] {2, 4, 10, 20}, 
      new double[] {30, 50, 80, 110}  
    )
  );

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
    Constants.INTAKE_MOTOR_ID,
    Constants.TILT_MOTOR_ID
  );

  private final FloorFeedSubsystem m_floorFeedSubsystem = new FloorFeedSubsystem();
  private final ClimbSubsystem m_climberSubsystem = new ClimbSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public void disableMotors() {
    m_shooterSubsystemJawbreaker.setIsShooting(false);
    m_shooterSubsystemJawbreaker.setIsFeeding(false);
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
    // Back button is 2 squares
    // Start button is 3 horizontal lines
    // POV is the D-pad

    // Shooter Speed Controls
    m_driverController.a().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.incrementShooterSpeed();
      m_shooterSubsystemJawbreaker.incrementShooterSpeed();
      m_shooterSubsystemTaylor.incrementShooterSpeed();
    }));
    m_driverController.b().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.decrementShooterSpeed();
      m_shooterSubsystemJawbreaker.decrementShooterSpeed();
      m_shooterSubsystemTaylor.decrementShooterSpeed();
    }));

    // Shooter and Feeder On/off Controls
    m_driverController.rightTrigger()
    .whileTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.setIsShooting(true);
      m_shooterSubsystemJawbreaker.setIsShooting(true);
      m_shooterSubsystemTaylor.setIsShooting(true);

      m_shooterSubsystemJohn.setIsFeeding(true);
      m_shooterSubsystemJawbreaker.setIsFeeding(true);
      m_shooterSubsystemTaylor.setIsFeeding(true);

      m_floorFeedSubsystem.setIsFeeding(true);
    }))
    .whileFalse(new InstantCommand(() -> {
      m_shooterSubsystemJohn.setIsShooting(false);
      m_shooterSubsystemJawbreaker.setIsShooting(false);
      m_shooterSubsystemTaylor.setIsShooting(false);

      m_shooterSubsystemJohn.setIsFeeding(false);
      m_shooterSubsystemJawbreaker.setIsFeeding(false);
      m_shooterSubsystemTaylor.setIsFeeding(false);

      m_floorFeedSubsystem.setIsFeeding(false);
    }));

    // Feeder Speed Controls
    m_driverController.y().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.incrementFeederSpeed();
      m_shooterSubsystemJawbreaker.incrementFeederSpeed();
      m_shooterSubsystemTaylor.incrementFeederSpeed();
    }));
    m_driverController.x().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.decrementFeederSpeed();
      m_shooterSubsystemJawbreaker.decrementFeederSpeed();
      m_shooterSubsystemTaylor.decrementFeederSpeed();
    }));

    // Linear Actuator Controls, 0.0-1.0 (total length)
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.setActuatorPosition(m_shooterSubsystemJohn.getActuatorPosition() - ACTUATOR_STEP);
      m_shooterSubsystemJawbreaker.setActuatorPosition(m_shooterSubsystemJawbreaker.getActuatorPosition() - ACTUATOR_STEP);
      m_shooterSubsystemTaylor.setActuatorPosition(m_shooterSubsystemTaylor.getActuatorPosition() - ACTUATOR_STEP);
    }));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.setActuatorPosition(m_shooterSubsystemJohn.getActuatorPosition() + ACTUATOR_STEP);
      m_shooterSubsystemJawbreaker.setActuatorPosition(m_shooterSubsystemJawbreaker.getActuatorPosition() + ACTUATOR_STEP);
      m_shooterSubsystemTaylor.setActuatorPosition(m_shooterSubsystemTaylor.getActuatorPosition() + ACTUATOR_STEP);
    }));

    // PID Tuning Controls for all shooter motors
    m_driverController.start().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.shootPIDMotor.putPIDF();
      m_shooterSubsystemJawbreaker.shootPIDMotor.putPIDF();
      m_shooterSubsystemTaylor.shootPIDMotor.putPIDF();

    }));
    m_driverController.back().onTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.shootPIDMotor.fetchPIDFFromDashboard();
      m_shooterSubsystemJawbreaker.shootPIDMotor.fetchPIDFFromDashboard();
      m_shooterSubsystemTaylor.shootPIDMotor.fetchPIDFFromDashboard();
    }));

    // Intake speed controls with d-pad up and down
    m_driverController.povUp().onTrue(new InstantCommand(() -> {
      m_intakeSubsystem.incrementIntakeSpeed();
    }));
    m_driverController.povDown().onTrue(new InstantCommand(() -> {
      m_intakeSubsystem.decrementIntakeSpeed();
    }));

    // Intake on/off control with left trigger
    m_driverController.leftTrigger()
    .whileTrue(new InstantCommand(() -> {
      m_intakeSubsystem.setIsIntaking(true);
    }))
    .whileFalse(new InstantCommand(() -> {
      m_intakeSubsystem.setIsIntaking(false);
    }));

    // TODO: Bind climb controls

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
