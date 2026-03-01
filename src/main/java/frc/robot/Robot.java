// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.auto.AutoCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  
  @NotLogged
  private Command m_autonomousCommand;

  @Logged
  private final RobotContainer m_robotContainer;

  @NotLogged
  private final Field2d m_botField;
  @NotLogged
  private final Field2d m_objectField;
  @NotLogged
  private final Field2d m_limelightField;

  // Logged - current alliance
  public static Alliance alliance = Alliance.Red;

  @NotLogged
  private final Timer teleopTimer = new Timer();

  private boolean forceActiveHub = false;

  @Logged(name = "Odometry translation error")
  @SuppressWarnings("unused")
  private double translationError;
  @Logged(name = "Odometry rotation error")
  @SuppressWarnings("unused")
  private double rotationError;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Start data logging to USB drive (if present)
    DataLogManager.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();
    updateRAlliance();
    this.m_botField = new Field2d();
    this.m_limelightField = new Field2d();
    this.m_objectField = new Field2d();

    // Configure Epilogue
    Epilogue.configure(config -> {
      config.minimumImportance = Logged.Importance.DEBUG;
      config.root = "Telemetry";
    });

    // Bind Epilogue logger - runs at robot frequency, offset by half phase
    Epilogue.bind(this);

    addPeriodic(this::sendFields, 0.080); // 80ms is every 4 loops
    addPeriodic(Robot::updateRAlliance, 0.5);
    addPeriodic(() -> SmartDashboard.putNumber("Distance to Target", 
      m_robotContainer.getDistanceToTarget(m_robotContainer.targetHub)), 0.1);
    addPeriodic(() -> SmartDashboard.putNumber("Degrees to Target", 
      m_robotContainer.getDegreesToTarget(m_robotContainer.targetHub)), 0.1);
    addPeriodic(this::updateRobotPose, 0.02);
    addPeriodic(m_robotContainer::updateDriverInputs, 0.02);
    m_robotContainer.autoChooser.onChange(this::updateFieldPaths);

    // Targets
    addPeriodic(this::updateTargetHub, 0.5);
    addPeriodic(this::updateTargetPassingZone, 0.04);
    addPeriodic(this::displayTarget, 0.1);

  }

  public void updateFieldPaths(AutoCommand auto) {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (auto != null) {
      m_botField.getObject("path").setPoses(auto.getAllProperFlippedPathPoses());
    } else {
      m_botField.getObject("path").setPoses();
    }
  }

  /**
   * Update the alliance color from the DriverStation.
   */
  public static void updateRAlliance() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private void displayTarget() {
    if (m_robotContainer.isInNeutralZone()) {
      m_botField.getObject("target").setPose(m_robotContainer.passTarget);
    }
    else {
      m_botField.getObject("target").setPose(m_robotContainer.compensatedTargetHub);
    }
  }

  private void updateTargetHub() {
    m_robotContainer.targetHub = alliance == Alliance.Red ? RobotContainer.RED_HUB : RobotContainer.BLUE_HUB;
  }

  /**
   * Updates the target passing zone
   */
  private void updateTargetPassingZone() {
    if (alliance == Alliance.Red) {
      if (m_robotContainer.drivetrain.getState().Pose.getY() > 4.035) {
        m_robotContainer.passTarget = RobotContainer.RED_ZONE_R;
      }
      else {
        m_robotContainer.passTarget = RobotContainer.RED_ZONE_L;
      }
    }
    else {
      if (m_robotContainer.drivetrain.getState().Pose.getY() > 4.035) {
        m_robotContainer.passTarget = RobotContainer.BLUE_ZONE_L;
      }
      else {
        m_robotContainer.passTarget = RobotContainer.BLUE_ZONE_R;
      }
    }
    m_objectField.setRobotPose(m_robotContainer.passTarget);
  }

  public void updateRobotPose(){
    var botState = m_robotContainer.drivetrain.getState();
    LimelightHelpers.SetRobotOrientation(Constants.SHOOTER_LIMELIGHT_NAME, 
        botState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    

    // Getting the robot's angular velocity in rotations per second
    double omegarps = Units.radiansToRotations(botState.Speeds.omegaRadiansPerSecond);
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.SHOOTER_LIMELIGHT_NAME);

    if (poseEstimate != null && poseEstimate.pose != null) {
      m_limelightField.setRobotPose(poseEstimate.pose);
      // push odometry error to dashboard
      this.translationError = m_robotContainer.drivetrain.getState().Pose.getTranslation().getDistance(poseEstimate.pose.getTranslation());
      this.rotationError = m_robotContainer.drivetrain.getState().Pose.getRotation().minus(poseEstimate.pose.getRotation()).getDegrees();
    } else {
      this.translationError = 0;
      this.rotationError = 0;
    }

    if (poseEstimate != null && poseEstimate.tagCount > 0 && Math.abs(omegarps) < 1.0) {
      m_robotContainer.drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
            VecBuilder.fill(.9, .9, 999999));
    }

    m_botField.setRobotPose(botState.Pose);
  }

    /**
   * Determines if the hub is active based on the current shift
   * @return true if the hub is active, false otherwise
   */
  public boolean isHubActive() {
    if (forceActiveHub) {
      return true;
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = teleopTimer.get();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active. Should add manual check.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
      default -> false;
    };

    boolean wonAuto = !shift1Active;

    double timeOfFlight = RobotContainer.TIME_OF_FLIGHT_INTERPOLATOR.interpolate(m_robotContainer.getDistanceToTarget(m_robotContainer.compensatedTargetHub));
    double minProcessingTime = 1.0; //seconds
    double maxProcessingTime = 2.0; //seconds
    double processingTime = 3.0; //seconds

    if (wonAuto) {
      if (matchTime < 10.0) {
        return true;
      } else if (matchTime < 35.0 - timeOfFlight - maxProcessingTime) { // substract timeOfFlight and (1s) processing time
        return false;
      } else if (matchTime < 60.0 - timeOfFlight - minProcessingTime + processingTime) { //minus time of flight minus (2s) processing time plus 3 seconds
        return true;
      } else if (matchTime < 85.0 - timeOfFlight - maxProcessingTime) { // substract timeOfFlight and (1s) processing time
        return false;
      } else if (matchTime < 110.0 - timeOfFlight - minProcessingTime + processingTime) { //minus time of flight minus (2s) processing time plus 3 seconds
        return true;
      } else if (matchTime >= 110.0 - timeOfFlight - maxProcessingTime) {
        return true;
      }
    }


    if (!wonAuto) {
      if (matchTime < 10.0) {
        return true;
      } else if (matchTime < 35.0 - timeOfFlight - minProcessingTime + processingTime) { // substract timeOfFlight and (2s) processing time plus 3 seconds
        return true;
      } else if (matchTime < 60.0 - timeOfFlight - maxProcessingTime) { //minus time of flight minus (1s) processing time
        return false;
      } else if (matchTime < 85.0 - timeOfFlight - minProcessingTime + processingTime) { // substract timeOfFlight and (2s) processing time plus 3 seconds
        return true;
      } else if (matchTime < 110.0 - timeOfFlight - maxProcessingTime) { //minus time of flight minus (1s) processing time
        return false;
      } else if (matchTime >= 110.0 - timeOfFlight - maxProcessingTime) {
        return true;
      }
    }

    return true; //idk error fix
  }

  @Override
  public void robotInit() {
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  public void sendFields() {
    SmartDashboard.putData("Bot Field", m_botField);
    SmartDashboard.putData("Limelight Field", m_limelightField);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableMotors();
  }

  @Override
  public void disabledPeriodic() {
    if (m_robotContainer.driverController.getHID().getStartButton() && m_autonomousCommand != null) {
      System.out.println("I did smth");
      AutoCommand autoCmd = (AutoCommand) m_autonomousCommand;
      m_robotContainer.drivetrain.resetPose(autoCmd.getProperFlippedStartingPose());
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    teleopTimer.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
