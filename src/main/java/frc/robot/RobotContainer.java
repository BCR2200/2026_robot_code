// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BlendAdamModeCmd;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DetectFuelCmd;
import frc.robot.commands.ShootAt;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.LeftBumpBack;
import frc.robot.commands.auto.LeftBumpToRight;
import frc.robot.commands.auto.LongLeftBumpBack;
import frc.robot.commands.auto.RightBumpBack;
import frc.robot.commands.auto.RightOutpost;
import frc.robot.commands.auto.TestOverrideAuto;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.drive.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FloorFeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {

  public boolean shootingAtHub = false;
  public boolean passing = false;

  public boolean redWonAuto = false;

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
  public Pose2d targetHub = RED_HUB;
  public Pose2d compensatedTargetHub = RED_HUB;

  public static final Pose2d RED_ZONE_L = new Pose2d(
    Distance.ofBaseUnits(13.8, Meters), 
    Distance.ofBaseUnits(1.5, Meters), 
    Rotation2d.kZero
  );
  public static final Pose2d RED_ZONE_R = new Pose2d(
    Distance.ofBaseUnits(13.8, Meters), 
    Distance.ofBaseUnits(6.0, Meters), 
    Rotation2d.kZero
  );
  public static final Pose2d BLUE_ZONE_L = new Pose2d(
    Distance.ofBaseUnits(2.2, Meters), 
    Distance.ofBaseUnits(6.0, Meters), 
    Rotation2d.kZero
  );
  public static final Pose2d BLUE_ZONE_R = new Pose2d(
    Distance.ofBaseUnits(2.2, Meters),
    Distance.ofBaseUnits(1.5, Meters),
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

  @NotLogged
  public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.8;
  @NotLogged
  public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  @NotLogged
  public final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  @NotLogged
  public final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  @NotLogged
  public final SwerveRequest.FieldCentricFacingAngle driveFCFA = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  @NotLogged
  public final SwerveRequest.FieldCentricFacingAngle driveFCFAVelocityMode = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity);

  @NotLogged
  private final Telemetry logger = new Telemetry(MaxSpeed);

  @NotLogged
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @NotLogged
  final SendableChooser<AutoCommand> autoChooser;

  @NotLogged
  private static final double ACTUATOR_STEP = 0.05;
  @NotLogged
  private static final int climbFinalCurrentLimit = 20;
  @NotLogged
  private static final int climbInitialCurrentLimit = 5;
  @NotLogged
  private static final int floorCurrentLimit = 30;
  @NotLogged
  private static final int intakeCurrentLimit = 80;
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

  private static final Interpolator HOOD_INTERPOLATOR = new Interpolator(
                  new double[] {0.966, 2.01, 3.00, 4.00, 5.00},
                  new double[] {0.050, 0.40, 0.65, 0.90, 1.0}
  );
  private static final Interpolator SHOTER_WEEL_VELOSITY_INTERPOLATOR = new Interpolator(
                  new double[] {0.966, 2.01, 3.00, 4.00, 5.00},
                  new double[] {50, 54, 58, 64, 73}
  );
  public static final Interpolator TIME_OF_FLIGHT_INTERPOLATOR = new Interpolator( // Placeholders for time of flight based on distance to target
                  new double[] {0.992, 2.01, 3.00, 4.00, 5.00},
                  new double[] {0.90, 0.96, 1.00, 1.02, 1.15}
  );

  // Subsystems - logged via their @Logged annotations
  @Logged(name = "John")
  public final ShooterSubsystem shooterSubsystemJohn = new ShooterSubsystem( "John",
          Constants.JOHN_SHOOTER_MOTOR_ID, Constants.JOHN_FEEDER_MOTOR_ID, Constants.JOHN_BEAMBREAK_CHANNEL, Constants.JOHN_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          HOOD_INTERPOLATOR,
          SHOTER_WEEL_VELOSITY_INTERPOLATOR,
          TIME_OF_FLIGHT_INTERPOLATOR,
          false, this
  );

  @Logged(name = "Jawbreaker")
  public final ShooterSubsystem shooterSubsystemJawbreaker = new ShooterSubsystem( "Jawbreaker",
          Constants.JAWBREAKER_SHOOTER_MOTOR_ID, Constants.JAWBREAKER_FEEDER_MOTOR_ID, Constants.JAWBREAKER_BEAMBREAK_CHANNEL, Constants.JAWBREAKER_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          HOOD_INTERPOLATOR,
          SHOTER_WEEL_VELOSITY_INTERPOLATOR,
          TIME_OF_FLIGHT_INTERPOLATOR,
          false, this
  );

  @Logged(name = "Taylor")
  public final ShooterSubsystem shooterSubsystemTaylor = new ShooterSubsystem( "Taylor",
          Constants.TAYLOR_SHOOTER_MOTOR_ID, Constants.TAYLOR_FEEDER_MOTOR_ID, Constants.TAYLOR_BEAMBREAK_CHANNEL, Constants.TAYLOR_LINEAR_ACTUATOR_CHANNEL, shooterCurrentLimit, feederCurrentLimit,
          HOOD_INTERPOLATOR,
          SHOTER_WEEL_VELOSITY_INTERPOLATOR,
          TIME_OF_FLIGHT_INTERPOLATOR,
          true, this
  );

  @Logged(name = "FloorFeed")
  private final FloorFeedSubsystem floorFeedSubsystem = new FloorFeedSubsystem(floorCurrentLimit, shooterSubsystemJohn, shooterSubsystemJawbreaker, shooterSubsystemTaylor);

  @Logged(name = "Climber")
  public final ClimbSubsystem climberSubsystem = new ClimbSubsystem(climbInitialCurrentLimit, climbFinalCurrentLimit);

  @Logged(name = "Intake")
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
          Constants.INTAKE_MOTOR_ID,
          Constants.TILT_MOTOR_ID,
          intakeCurrentLimit, tiltCurrentLimit,
          floorFeedSubsystem
  );

  @NotLogged
  public final CommandXboxController driverController =
          new CommandXboxController(OperatorConstants.kDriverControllerPort);

  @NotLogged
  public final CommandXboxController coDriverController =
          new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new EventTrigger("IntakeDown").whileTrue(new InstantCommand(() -> {
              intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMaxExtensionPos);
            }));
    new EventTrigger("IntakeUp").whileTrue(new InstantCommand(() -> {
              intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMinExtensionPos);
            }));
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
    configureCoDriverBindings();
    configureDrivetrainBindings();
    

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("TestOverrideAuto", new TestOverrideAuto(this, drivetrain, driveRC));
    autoChooser.addOption("RightOutpost", new RightOutpost(this, drivetrain, driveRC));
    autoChooser.addOption("RightBumpBack", new RightBumpBack(this, drivetrain, driveRC));
    autoChooser.addOption("LeftBumpBack", new LeftBumpBack(this, drivetrain, driveRC));
    autoChooser.addOption("LongLeftBumpBack", new LongLeftBumpBack(this, drivetrain, driveRC));
    autoChooser.addOption("LeftBumpToRight", new LeftBumpToRight(this, drivetrain, driveRC));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void disableMotors() {
    shooterSubsystemJohn.setIsShooting(false);
    shooterSubsystemJawbreaker.setIsShooting(false);
    shooterSubsystemTaylor.setIsShooting(false);
    shooterSubsystemJohn.setIsFeeding(false);
    shooterSubsystemJawbreaker.setIsFeeding(false);
    shooterSubsystemTaylor.setIsFeeding(false);
    intakeSubsystem.setIsIntaking(false);
    intakeSubsystem.setIsGoingUp(false);
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

  public void calculateCompensatedTargetHub() {
    compensatedTargetHub = targetHub;

    ChassisSpeeds speeds = drivetrain.getState().Speeds;
    Rotation2d angle = drivetrain.getState().Pose.getRotation();
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, angle);

    double robotVelocityX = fieldSpeeds.vxMetersPerSecond;
    double robotVelocityY = fieldSpeeds.vyMetersPerSecond;

    if (Math.abs(robotVelocityX) > 0.1 || Math.abs(robotVelocityY) > 0.1) {
      
      for (int i = 0; i < 20; i++) {
        double distance = getDistanceToTarget(compensatedTargetHub);
        double timeOfFlight = TIME_OF_FLIGHT_INTERPOLATOR.interpolate(distance);
        double offsetX = robotVelocityX * timeOfFlight;
        double offsetY = robotVelocityY * timeOfFlight;

        compensatedTargetHub = new Pose2d(
            targetHub.getX() - offsetX,
            targetHub.getY() - offsetY,
            Rotation2d.kZero);
      }
    }
  }


  /**
   * Checks if the robot is outside its alliance zone, which is past 4.625 (on
   * blue) and before 11.916 meters (on red) from the starting wall.
   * @return true if the robot is outside its alliance zone, false otherwise
   */
  public boolean isOutsideAllianceZone() {
    double distance = drivetrain.getState().Pose.getX();
    // l = 4.625 r = 11.916
    if (Robot.alliance == Alliance.Red) {
      return distance < 11.916;
    } else {
      return distance > 4.625;
    }
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

    // outtake
    driverController.leftBumper().whileTrue(new InstantCommand(() -> {
      intakeSubsystem.setIsOuttaking(true);
      floorFeedSubsystem.setIsOuttaking(true);
      intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMaxExtensionPos);
    })).whileFalse(new InstantCommand(() -> {
      intakeSubsystem.setIsOuttaking(false);
      floorFeedSubsystem.setIsOuttaking(false);
    }));

    // intake
    driverController.leftTrigger()
    .whileTrue(new InstantCommand(() -> {
      intakeSubsystem.setIsIntaking(true);
      intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMaxExtensionPos);
    }))
    .whileFalse(new InstantCommand(() -> {
      intakeSubsystem.setIsIntaking(false);
    }));
    driverController.rightBumper().whileTrue(new BlendAdamModeCmd(this));
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

    // climb
    driverController.b().whileTrue(new ClimbCommand(this, true)); // right
    driverController.x().whileTrue(new ClimbCommand(this, false)); // left

    // bring intake up
    driverController.a().onTrue(new InstantCommand(() -> {
      intakeSubsystem.setIsGoingUp(true);
    })).onFalse(new InstantCommand(() -> {
      intakeSubsystem.setIsGoingUp(false);
    }));

    driverController.y().onTrue(new InstantCommand(() -> {
      shooterSubsystemJohn.updateParameters();
      shooterSubsystemJawbreaker.updateParameters();
      shooterSubsystemTaylor.updateParameters();
      climberSubsystem.updateParameters();
      floorFeedSubsystem.updateParameters();
      intakeSubsystem.updateParameters();
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

    driverController.start().whileTrue(new InstantCommand(() -> {})); // Used in disabled for syncing autos
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
        updateDriverInputs();
          return driveFC.withVelocityX(-driverY * MaxSpeed) // Drive forward with negative Y
                  .withVelocityY(-driverX * MaxSpeed) // Drive left with negative X
                  .withRotationalRate(-driverRot * MaxAngularRate) // Drive counterclockwise with negative X
                  .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective); // With operator perspective
        
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

  private void configureCoDriverBindings() {

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
    drivetrain.seedFieldCentric();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public AutoCommand getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }


}
