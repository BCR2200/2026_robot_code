// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DetectFuelCmd;
import frc.robot.commands.JustShootCmd;
import frc.robot.commands.PassCmd;
import frc.robot.commands.SnapTowardsGoalCmd;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.drive.TunerConstantsComp;
import frc.robot.drive.TunerConstantsPrac;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {

  @NotLogged
  public final static double MaxSpeed = TunerConstantsComp.kSpeedAt12Volts.in(MetersPerSecond) * 0.2;
  @NotLogged
  public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Logged automatically
  public boolean isManualMode = false;

  @NotLogged
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  @NotLogged
  private final Telemetry logger = new Telemetry(MaxSpeed);

  @NotLogged
  public final CommandSwerveDrivetrain drivetrain = TunerConstantsPrac.createDrivetrain();

  @NotLogged
  private static final double ACTUATOR_STEP = 0.05;
  @NotLogged
  private static final int climbCurrentLimit = 30;
  @NotLogged
  private static final int floorCurrentLimit = 30;
  @NotLogged
  private static final int intakeCurrentLimit = 60;
  @NotLogged
  private static final int tiltCurrentLimit = 10; // Normally 25
  @NotLogged
  private static final int shooterCurrentLimit = 60;
  @NotLogged
  private static final int feederCurrentLimit = 30;

  // Subsystems - logged via their @Logged annotations
  @Logged(name = "John")
  private final ShooterSubsystem m_shooterSubsystemJohn = new ShooterSubsystem( "John",
          Constants.JOHN_SHOOTER_MOTOR_ID, Constants.JOHN_FEEDER_MOTOR_ID, Constants.JOHN_BEAMBREAK_CHANNEL, Constants.JOHN_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          new Interpolator( // Placeholders for shoot hood angles
                  new double[] {2, 4, 10, 20},
                  new double[] {1, 0.75, 0.5, 0.25}
          ),
          new Interpolator( // Placeholders for shoot wheel velocities
                  new double[] {2, 4, 10, 20},
                  new double[] {30, 50, 80, 110}
          ),
          false
  );

  @Logged(name = "Jawbreaker")
  private final ShooterSubsystem m_shooterSubsystemJawbreaker = new ShooterSubsystem( "Jawbreaker",
          Constants.JAWBREAKER_SHOOTER_MOTOR_ID, Constants.JAWBREAKER_FEEDER_MOTOR_ID, Constants.JAWBREAKER_BEAMBREAK_CHANNEL, Constants.JAWBREAKER_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          new Interpolator( // Placeholders for shoot hood angles
                  new double[] {2, 4, 10, 20},
                  new double[] {1, 0.75, 0.5, 0.25}
          ),
          new Interpolator( // Placeholders for shoot wheel velocities
                  new double[] {2, 4, 10, 20},
                  new double[] {30, 50, 80, 110}
          ),
          false
  );

  @Logged(name = "Taylor")
  private final ShooterSubsystem m_shooterSubsystemTaylor = new ShooterSubsystem( "Taylor",
          Constants.TAYLOR_SHOOTER_MOTOR_ID, Constants.TAYLOR_FEEDER_MOTOR_ID, Constants.TAYLOR_BEAMBREAK_CHANNEL, Constants.TAYLOR_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          new Interpolator( // Placeholders for shoot hood angles
                  new double[] {2, 4, 10, 20},
                  new double[] {1, 0.75, 0.5, 0.25}
          ),
          new Interpolator( // Placeholders for shoot wheel velocities
                  new double[] {2, 4, 10, 20},
                  new double[] {30, 50, 80, 110}
          ),
          true
  );

  @Logged(name = "FloorFeed")
  private final FloorFeedSubsystem m_floorFeedSubsystem = new FloorFeedSubsystem(floorCurrentLimit, m_shooterSubsystemJohn, m_shooterSubsystemJawbreaker, m_shooterSubsystemTaylor);

  @Logged(name = "Climber")
  private final ClimbSubsystem m_climberSubsystem = new ClimbSubsystem(climbCurrentLimit);

  @Logged(name = "Intake")
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
          Constants.INTAKE_MOTOR_ID,
          Constants.TILT_MOTOR_ID,
          intakeCurrentLimit, tiltCurrentLimit,
          m_floorFeedSubsystem
  );

  @NotLogged
  private final CommandXboxController m_driverController =
          new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDrivetrainBindings();
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

    m_driverController.leftBumper().whileTrue(new DetectFuelCmd(drivetrain));
    m_driverController.leftTrigger()
            .whileTrue(new InstantCommand(() -> {
              m_intakeSubsystem.setIsIntaking(true);
            }))
            .whileFalse(new InstantCommand(() -> {
              m_intakeSubsystem.setIsIntaking(false);
            }));
    m_driverController.rightBumper().whileTrue(new PassCmd(drivetrain, m_shooterSubsystemJohn, m_shooterSubsystemJawbreaker, m_shooterSubsystemTaylor, m_floorFeedSubsystem)); // TODONE
    // m_driverController.rightTrigger().onTrue(new SnapTowardsGoalCmd(drivetrain).andThen(JustShootCmd.getStartCommand(m_shooterSubsystemJohn, m_shooterSubsystemJawbreaker, m_shooterSubsystemTaylor)))
    //                                  .onFalse(JustShootCmd.getStopCommand(m_shooterSubsystemJohn, m_shooterSubsystemJawbreaker, m_shooterSubsystemTaylor)); // TODO: implement shoot-to-goal
    m_driverController.rightTrigger().whileTrue(new JustShootCmd(m_shooterSubsystemJohn, m_shooterSubsystemJawbreaker, m_shooterSubsystemTaylor)); // TODO: implement shoot-to-goal


    m_driverController.b().whileTrue(new InstantCommand(() -> {})); // TODO: implement right climb
    m_driverController.a().whileTrue(new InstantCommand(() -> {
      m_shooterSubsystemJohn.updateParameters();
      m_shooterSubsystemJawbreaker.updateParameters();
      m_shooterSubsystemTaylor.updateParameters();
      m_climberSubsystem.updateParameters();
      m_floorFeedSubsystem.updateParameters();
      m_intakeSubsystem.updateParameters();
    }));
    m_driverController.x().whileTrue(new InstantCommand(() -> {})); // TODO: implement left climb
    m_driverController.y().onTrue(new InstantCommand(() -> updateDrivetrainRobotPerspective()));

    m_driverController.povLeft().onTrue(new InstantCommand(() -> {m_intakeSubsystem.setTiltPosition(m_intakeSubsystem.getTiltPosition() + 4);})); // TODO: implement reset alliance - possibly reseed field-centric?
    m_driverController.povRight().onTrue(new InstantCommand(() -> {m_intakeSubsystem.setTiltPosition(m_intakeSubsystem.getTiltPosition() - 3);})); // TODO: implement reset facing angle
    m_driverController.povUp().whileTrue(new InstantCommand(() -> {
      if (isManualMode) {
        m_shooterSubsystemJohn.setActuatorTargetPosition(m_shooterSubsystemJohn.getActuatorPosition() + ACTUATOR_STEP);
        m_shooterSubsystemJawbreaker.setActuatorTargetPosition(m_shooterSubsystemJawbreaker.getActuatorPosition() + ACTUATOR_STEP);
        m_shooterSubsystemTaylor.setActuatorTargetPosition(m_shooterSubsystemTaylor.getActuatorPosition() + ACTUATOR_STEP);
      }
    }));
    m_driverController.povDown().whileTrue(new InstantCommand(() -> {
      if (isManualMode) {
        m_shooterSubsystemJohn.setActuatorTargetPosition(m_shooterSubsystemJohn.getActuatorPosition() - ACTUATOR_STEP);
        m_shooterSubsystemJawbreaker.setActuatorTargetPosition(m_shooterSubsystemJawbreaker.getActuatorPosition() - ACTUATOR_STEP);
        m_shooterSubsystemTaylor.setActuatorTargetPosition(m_shooterSubsystemTaylor.getActuatorPosition() - ACTUATOR_STEP);
      }
    }));

    m_driverController.start().whileTrue(new InstantCommand(() -> {})); // TODO: implement unjam
    m_driverController.back().whileTrue(new InstantCommand(() -> this.isManualMode = !this.isManualMode));

  }

  private void configureDrivetrainBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
            )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // reset the field-centric heading on back button press
    m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Note: leftBumper DetectFuelCmd is bound in configureBindings()

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void updateDrivetrainRobotPerspective() {
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Robot.LIMELIGHTS[0]);
    if (llMeasurement != null){
      drivetrain.resetPose(llMeasurement.pose);
    }
    Rotation2d forward;
    if (Robot.alliance == Alliance.Red) {
      forward = new Rotation2d(Math.PI);
    } else {
      forward = new Rotation2d(0);
    }
    drivetrain.setOperatorPerspectiveForward(forward);

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
