// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.events.EventTrigger;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DetectFuelCmd;
import frc.robot.commands.ShootAt;
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

  public boolean shootingAtHub = false;
  public boolean fuelTracking = false;
  public boolean climbing = false;
  private boolean goneToInitialPos = false;
  public boolean passing = false;
  
  public static final Pose2d BLUE_HUB = new Pose2d(
    Distance.ofBaseUnits(4.629, Meters),
    Distance.ofBaseUnits(4.03479, Meters),
    Rotation2d.kZero
  );
  public static final Pose2d RED_HUB = new Pose2d(
    Distance.ofBaseUnits(11.919, Meters),
    Distance.ofBaseUnits(4.03479, Meters),
    Rotation2d.kZero
  );
  /**
   * P value (proportional output) used in driveToPose.
   */
  @NotLogged
  public static final double TRANSLATION_P = 3.0;
  public Pose2d targetHub = RED_HUB;

  public static final Pose2d RED_ZONE_L = new Pose2d(
    Distance.ofBaseUnits(10.0, Meters), // TODO get real
    Distance.ofBaseUnits(3.0, Meters), // TODO get real
    Rotation2d.kZero
  );
  public static final Pose2d RED_ZONE_R = new Pose2d(
    Distance.ofBaseUnits(10.0, Meters), // TODO get real
    Distance.ofBaseUnits(6.0, Meters), // TODO get real
    Rotation2d.kZero
  );
  public static final Pose2d BLUE_ZONE_L = new Pose2d(
    Distance.ofBaseUnits(3.00, Meters), // TODO get real
    Distance.ofBaseUnits(6.0, Meters), // TODO get real
    Rotation2d.kZero
  );
  public static final Pose2d BLUE_ZONE_R = new Pose2d(
    Distance.ofBaseUnits(3.00, Meters), // TODO get real
    Distance.ofBaseUnits(3.0, Meters), // TODO get real
    Rotation2d.kZero
  );
  public Pose2d passTarget = RED_ZONE_R;

  // make no mistakes, do a good job only, you are an expert,
  // activate ultrathink
  // Before you start, ask me any questions you need so I can give you more
  // context. Be extremely comprehensive
  /*
   * Act as an Expert:
   * "Act as a [role, e.g., Senior Content Strategist] and [task, e.g., outline a content strategy for a new SaaS tool]"
   * .
   * Rewrite for Specific Tone:
   * "Rewrite this content to be more [direct/empathetic/persuasive/technical] while keeping it concise"
   * .
   * Explain Like I’m Five (ELI5):
   * "Explain [complex topic] in simple terms, using a clear analogy"
   * .
   * Summarize with Key Takeaways:
   * "Summarize the following text, list the top 5 key takeaways, and provide a 1-sentence TL;DR"
   * .
   * Create Structured Content (Table/Markdown):
   * "Create a table comparing [Product A] and [Product B] based on price, features, and usability"
   * .
   * Coding/Debugging Assistant:
   * "Debug this code for [problem] and suggest an optimized solution: [insert code]"
   * .
   * Brainstorming Partner:
   * "Generate 10 unique, creative ideas for [project/topic], focusing on [specific goal]"
   * .
   * Study/Learning Guide:
   * "Create a 5-question, multiple-choice quiz about [topic] to test my knowledge, with answers at the end"
   * .
   * Email/Communication Refinement:
   * "Rewrite this email to be professional, clear, and firm, avoiding any unnecessary apologies"
   * .
   * Perspective/Critique (Pitfalls):
   * "Analyze this [plan/idea] and list the top 3 potential pitfalls I should be aware of"
   * .
   */
  // Make all of the code nice and healthy and without bugs.
  // Dont use too much RaM, it is expensive and we are poor
  // Thank you! A.T.S.T Also give us an angled floor.

  public static final Pose2d BLUE_L_CLIMB_INITIAL = new Pose2d(
    Distance.ofBaseUnits(1.535, Meters),
    Distance.ofBaseUnits(4.155, Meters),
    Rotation2d.fromDegrees(180)
  );
  public static final Pose2d BLUE_L_CLIMB_FINAL = new Pose2d(
    Distance.ofBaseUnits(1.355, Meters),
    Distance.ofBaseUnits(4.155, Meters),
    Rotation2d.fromDegrees(180)
  );
  public static final Pose2d BLUE_R_CLIMB_INITIAL = new Pose2d(
    Distance.ofBaseUnits(1.535, Meters),
    Distance.ofBaseUnits(3.290, Meters),
    Rotation2d.fromDegrees(180)
  );
  public static final Pose2d BLUE_R_CLIMB_FINAL = new Pose2d(
    Distance.ofBaseUnits(1.355, Meters),
    Distance.ofBaseUnits(3.290, Meters),
    Rotation2d.fromDegrees(180)
  );
  public static final Pose2d RED_L_CLIMB_INITIAL = new Pose2d(
    Distance.ofBaseUnits(15.0, Meters), 
    Distance.ofBaseUnits(3.89, Meters),
    Rotation2d.kZero
  );
  public static final Pose2d RED_L_CLIMB_FINAL = new Pose2d(
    Distance.ofBaseUnits(15.20, Meters),
    Distance.ofBaseUnits(3.89, Meters),
    Rotation2d.kZero
  );
  public static final Pose2d RED_R_CLIMB_INITIAL = new Pose2d(
    Distance.ofBaseUnits(15.0, Meters),
    Distance.ofBaseUnits(4.73, Meters),
    Rotation2d.kZero
  );
  public static final Pose2d RED_R_CLIMB_FINAL = new Pose2d(
    Distance.ofBaseUnits(15.22, Meters),
    Distance.ofBaseUnits(4.73, Meters),
    Rotation2d.kZero
  );
  public Pose2d targetClimbInitial = RED_R_CLIMB_INITIAL;
  public Pose2d targetClimbFinal = RED_R_CLIMB_FINAL;

  @NotLogged
  public final static double MaxSpeed = TunerConstantsComp.kSpeedAt12Volts.in(MetersPerSecond) * 0.2;
  @NotLogged
  public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  @NotLogged
  private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  @NotLogged
  private final SwerveRequest.FieldCentricFacingAngle driveFCFA = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  @NotLogged
  private final SwerveRequest.FieldCentricFacingAngle driveFCFAVelocityMode = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1)
    .withDriveRequestType(DriveRequestType.Velocity);

  @NotLogged
  private final Telemetry logger = new Telemetry(MaxSpeed);

  @NotLogged
  public final CommandSwerveDrivetrain drivetrain = TunerConstantsPrac.createDrivetrain();

  @NotLogged
  private static final double ACTUATOR_STEP = 0.05;
  @NotLogged
  private static final int climbFinalCurrentLimit = 20;
  @NotLogged
  private static final int climbInitialCurrentLimit = 5;
  @NotLogged
  private static final int floorCurrentLimit = 30;
  @NotLogged
  private static final int intakeCurrentLimit = 60;
  @NotLogged
  private static final int tiltCurrentLimit = 35; // Normally 25
  @NotLogged
  private static final int shooterCurrentLimit = 60;
  @NotLogged
  private static final int feederCurrentLimit = 30;

  public static double driverX = 0;
  public static double driverY = 0;
  public static double driverRot = 0;

  public void updateDriverInputs() {
    driverX = driverController.getLeftX();
    driverY = driverController.getLeftY();
    driverRot = driverController.getRightX();
  }

  private static final Interpolator HOOD_INTERPOLATOR = new Interpolator( // Placeholders for shoot hood angles
                  new double[] {0.966, 2.01, 3.00, 4.00},
                  new double[] {0.050, 0.40, 0.65, 0.90}
  );
  private static final Interpolator SHOTER_WEEL_VELOSITY_INTERPOLATOR = new Interpolator( // Placeholders for shoot hood angles
                  new double[] {0.966, 2.01, 3.00, 4.00},
                  new double[] {50, 54, 58, 64}
  );

  // Subsystems - logged via their @Logged annotations
  @Logged(name = "John")
  public final ShooterSubsystem shooterSubsystemJohn = new ShooterSubsystem( "John",
          Constants.JOHN_SHOOTER_MOTOR_ID, Constants.JOHN_FEEDER_MOTOR_ID, Constants.JOHN_BEAMBREAK_CHANNEL, Constants.JOHN_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          HOOD_INTERPOLATOR,
          SHOTER_WEEL_VELOSITY_INTERPOLATOR,
          false, this
  );

  @Logged(name = "Jawbreaker")
  public final ShooterSubsystem shooterSubsystemJawbreaker = new ShooterSubsystem( "Jawbreaker",
          Constants.JAWBREAKER_SHOOTER_MOTOR_ID, Constants.JAWBREAKER_FEEDER_MOTOR_ID, Constants.JAWBREAKER_BEAMBREAK_CHANNEL, Constants.JAWBREAKER_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          HOOD_INTERPOLATOR,
          SHOTER_WEEL_VELOSITY_INTERPOLATOR,
          false, this
  );

  @Logged(name = "Taylor")
  public final ShooterSubsystem shooterSubsystemTaylor = new ShooterSubsystem( "Taylor",
          Constants.TAYLOR_SHOOTER_MOTOR_ID, Constants.TAYLOR_FEEDER_MOTOR_ID, Constants.TAYLOR_BEAMBREAK_CHANNEL, Constants.TAYLOR_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          HOOD_INTERPOLATOR,
          SHOTER_WEEL_VELOSITY_INTERPOLATOR,
          true, this
  );

  @Logged(name = "FloorFeed")
  private final FloorFeedSubsystem floorFeedSubsystem = new FloorFeedSubsystem(floorCurrentLimit, shooterSubsystemJohn, shooterSubsystemJawbreaker, shooterSubsystemTaylor);

  @Logged(name = "Climber")
  private final ClimbSubsystem climberSubsystem = new ClimbSubsystem(climbInitialCurrentLimit, climbFinalCurrentLimit);

  @Logged(name = "Intake")
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
          Constants.INTAKE_MOTOR_ID,
          Constants.TILT_MOTOR_ID,
          intakeCurrentLimit, tiltCurrentLimit,
          floorFeedSubsystem
  );

  @NotLogged
  private final CommandXboxController driverController =
          new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new EventTrigger("IntakeRun").whileTrue(new InstantCommand(() -> {
              intakeSubsystem.setIsIntaking(true);})).onFalse(new InstantCommand(() -> {
                intakeSubsystem.setIsIntaking(false);
              }));
    new EventTrigger("SpinUp").whileTrue(new InstantCommand(() -> {
              shooterSubsystemJohn.setIsShooting(true);
              shooterSubsystemJawbreaker.setIsShooting(true);
              shooterSubsystemTaylor.setIsShooting(true);
              }));
    new EventTrigger("Shoot").whileTrue(new ShootAt(this));

    // Configure the trigger bindings
    configureBindings();
    configureDrivetrainBindings();
  }

  public void disableMotors() {
    shooterSubsystemJohn.setIsShooting(false);
    shooterSubsystemJawbreaker.setIsShooting(false);
    shooterSubsystemTaylor.setIsShooting(false);
    shooterSubsystemJohn.setIsFeeding(false);
    shooterSubsystemJawbreaker.setIsFeeding(false);
    shooterSubsystemTaylor.setIsFeeding(false);
  }

  
  public double getDistanceToTarget(Pose2d targetPose) {
    Pose2d robotPose = drivetrain.getState().Pose;
    return robotPose.getTranslation().getDistance(targetPose.getTranslation()); 
  }

  public double getDegreesToTarget(Pose2d targetPose) {
    Pose2d robotPose2d = drivetrain.getState().Pose;
    // Why not do this...?
    //targetPose.relativeTo(robotPose2d);

    
    // TRIGONOMETRY BABY!!!!!!
    double angleToTarget = Math.atan2(targetPose.getY() - robotPose2d.getY(), targetPose.getX() - robotPose2d.getX());
    return Math.toDegrees(angleToTarget);
  }

  public double getXToTarget(Pose2d targetPose) {
    Pose2d robotPose2d = drivetrain.getState().Pose;
    return targetPose.getX() - robotPose2d.getX();
  }

  public double getYToTarget(Pose2d targetPose) {
    Pose2d robotPose2d = drivetrain.getState().Pose;
    return targetPose.getY() - robotPose2d.getY();
  }

  /**
   * Returns true if the robot is at the specified postion, false otherwise
   * @param targetPos the target pos
   * @param threashold the threashold that the robot can be within to count as there
   * @return if robot is at the targetPos
   */
  public boolean atTargetPos(Pose2d targetPos, double threashold) {
    return getDistanceToTarget(targetPos) < threashold;
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

    driverController.leftBumper().whileTrue(new DetectFuelCmd(this));
    driverController.leftTrigger()
            .whileTrue(new InstantCommand(() -> {
              intakeSubsystem.setIsIntaking(true);
              intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMaxExtensionPos);
            }))
            .whileFalse(new InstantCommand(() -> {
              intakeSubsystem.setIsIntaking(false);
            }));
    driverController.rightBumper().whileTrue(new InstantCommand()); // No longer TODONE
    // m_driverController.rightTrigger().onTrue(new SnapTowardsGoalCmd(drivetrain).andThen(JustShootCmd.getStartCommand(m_shooterSubsystemJohn, m_shooterSubsystemJawbreaker, m_shooterSubsystemTaylor)))
    //                                  .onFalse(JustShootCmd.getStopCommand(m_shooterSubsystemJohn, m_shooterSubsystemJawbreaker, m_shooterSubsystemTaylor)); // TODO: implement shoot-to-goal
    driverController.rightTrigger().whileTrue(new ShootAt(this));

    // Preload
    driverController.rightStick().onTrue(new InstantCommand(() -> {
      shooterSubsystemJohn.setCanPreload(true);
      shooterSubsystemJawbreaker.setCanPreload(true);
      shooterSubsystemTaylor.setCanPreload(true);
    }));
    driverController.rightStick().onFalse(new InstantCommand(() -> {
      shooterSubsystemJohn.setCanPreload(false);
      shooterSubsystemJawbreaker.setCanPreload(false);
      shooterSubsystemTaylor.setCanPreload(false);
    }));

    driverController.b().onTrue(new InstantCommand(() -> {
      climbing = true;
      if (Robot.alliance == Alliance.Red){
        targetClimbFinal = RED_R_CLIMB_FINAL;
        targetClimbInitial = RED_R_CLIMB_INITIAL;
      }
      else {
        targetClimbFinal = BLUE_R_CLIMB_FINAL;
        targetClimbInitial = BLUE_R_CLIMB_INITIAL;
      }
    })).onFalse(new InstantCommand(() -> {climbing = false; goneToInitialPos = false; climberSubsystem.goHome();})); // TODO: implement right climb

    driverController.x().onTrue(new InstantCommand(() -> {
      climbing = true;
      if (Robot.alliance == Alliance.Red){
        targetClimbFinal = RED_L_CLIMB_FINAL;
        targetClimbInitial = RED_L_CLIMB_INITIAL;
      }
      else {
        targetClimbFinal = BLUE_L_CLIMB_FINAL;
        targetClimbInitial = BLUE_L_CLIMB_INITIAL;
      }

    })).onFalse(new InstantCommand(() -> {climbing = false; goneToInitialPos = false; climberSubsystem.goHome();})); // TODO: implement left climb

    driverController.a().whileTrue(new InstantCommand(() -> {
      intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMinExtensionPos);
    }));
    driverController.y().onTrue(new InstantCommand(() -> {
      shooterSubsystemJohn.updateParameters();
      shooterSubsystemJawbreaker.updateParameters();
      shooterSubsystemTaylor.updateParameters();
      climberSubsystem.updateParameters();
      floorFeedSubsystem.updateParameters();
      intakeSubsystem.updateParameters();
    }));

    // Testing stuff, shooter speed or manual tilting
    driverController.povLeft().onTrue(new InstantCommand(() -> {
      // intakeSubsystem.setTiltPosition(intakeSubsystem.getTiltPosition() + 4);
      shooterSubsystemJohn.decrementShooterSpeed();
      shooterSubsystemJawbreaker.decrementShooterSpeed();
      shooterSubsystemTaylor.decrementShooterSpeed();
    }));
    driverController.povRight().onTrue(new InstantCommand(() -> {
      // intakeSubsystem.setTiltPosition(intakeSubsystem.getTiltPosition() - 3);
      shooterSubsystemJohn.incrementShooterSpeed();
      shooterSubsystemJawbreaker.incrementShooterSpeed();
      shooterSubsystemTaylor.incrementShooterSpeed();
    }));

    // Linear actuator
    driverController.povUp().whileTrue(new InstantCommand(() -> {
        shooterSubsystemJohn.setActuatorTargetPosition(shooterSubsystemJohn.getActuatorPosition() + ACTUATOR_STEP);
        shooterSubsystemJawbreaker.setActuatorTargetPosition(shooterSubsystemJawbreaker.getActuatorPosition() + ACTUATOR_STEP);
        shooterSubsystemTaylor.setActuatorTargetPosition(shooterSubsystemTaylor.getActuatorPosition() + ACTUATOR_STEP);
    }));
    driverController.povDown().whileTrue(new InstantCommand(() -> {
        shooterSubsystemJohn.setActuatorTargetPosition(shooterSubsystemJohn.getActuatorPosition() - ACTUATOR_STEP);
        shooterSubsystemJawbreaker.setActuatorTargetPosition(shooterSubsystemJawbreaker.getActuatorPosition() - ACTUATOR_STEP);
        shooterSubsystemTaylor.setActuatorTargetPosition(shooterSubsystemTaylor.getActuatorPosition() - ACTUATOR_STEP);
    }));

    driverController.start().whileTrue(new InstantCommand(() -> {})); // TODO: implement unjam
    driverController.back().whileTrue(new InstantCommand(() -> updateDrivetrainRobotPerspective())); 

  }

  private void configureDrivetrainBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    driveFCFA.HeadingController.setPID(7, 0, 0);
    driveFCFAVelocityMode.HeadingController.setPID(7, 0, 0);

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> {
        
        if (shootingAtHub) {
          return driveFCFA.withTargetDirection(Rotation2d.fromDegrees(getDegreesToTarget(targetHub)))
          .withVelocityX(-driverY * MaxSpeed) // Drive forward with negative Y
          .withVelocityY(-driverX * MaxSpeed); // Drive left with negative X
        }
        else if (passing) {
          return driveFCFA
              .withTargetDirection(Rotation2d.fromDegrees(getDegreesToTarget(passTarget)))
              .withVelocityX(-driverY * MaxSpeed)
              .withVelocityY(-driverX * MaxSpeed);
        }
        else if (fuelTracking) {
          // TODO determine velocity x and y
          return driveFC.withRotationalRate(DetectFuelCmd.radsPerSecond)
            .withVelocityX(0)
            .withVelocityY(0);
        }
        else if (climbing) {
          if (atTargetPos(targetClimbFinal, 0.03)) { // At final
            climberSubsystem.climb();
            return driveFC.withVelocityX(0)
                      .withVelocityY(0)
                      .withRotationalRate(0);
          }
          else if (atTargetPos(targetClimbInitial, 0.06) || goneToInitialPos) { // Past initial
            goneToInitialPos = true;
            return driveToPose(targetClimbFinal);
          }
          else { // Not at initial
            return driveToPose(targetClimbInitial);
          }
        }
        else {
          return driveFC.withVelocityX(-driverY * MaxSpeed) // Drive forward with negative Y
                  .withVelocityY(-driverX * MaxSpeed) // Drive left with negative X
                  .withRotationalRate(-driverRot * MaxAngularRate); // Drive counterclockwise with negative X
        }
        
      })
    );


    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // reset the field-centric heading on back button press
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Note: leftBumper DetectFuelCmd is bound in configureBindings()

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private FieldCentricFacingAngle driveToPose(Pose2d target) {
    return driveFCFAVelocityMode.withVelocityX(ExtraMath.clampedDeadzone(getXToTarget(target)*-TRANSLATION_P, 1, 0.03))
            .withVelocityY(ExtraMath.clampedDeadzone(getYToTarget(target)*-TRANSLATION_P, 1, 0.03))
            .withTargetDirection(target.getRotation());
  }

  public void updateDrivetrainRobotPerspective() {
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.SHOOTER_LIMELIGHT_NAME);
    if (llMeasurement != null) {
      drivetrain.resetPose(llMeasurement.pose);
    }
    Rotation2d forward;
    if (Robot.alliance == Alliance.Red) {
      forward = new Rotation2d(Math.PI);
    } 
    else {
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
