// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
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
    updateAlliance();
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

    // Example of periodic task. Calls this.updateTargetHub() every 0.5 seconds
    addPeriodic(this::updateTargetHub, 0.5);
    addPeriodic(this::sendFields, 0.080); // 80ms is every 4 loops
    addPeriodic(Robot::updateAlliance, 0.5);
    addPeriodic(() -> SmartDashboard.putNumber("Distance to Target", 
      m_robotContainer.getDistanceToTarget(m_robotContainer.targetHub)), 0.1);
    addPeriodic(() -> SmartDashboard.putNumber("Degrees to Target", 
      m_robotContainer.getDegreesToTarget(m_robotContainer.targetHub)), 0.1);
    addPeriodic(() -> updateRobotPose(), 0.02);
  }

  /**
   * Update the alliance color from the DriverStation.
   */
  public static void updateAlliance() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Red);
  }


  private void updateTargetHub() {
    m_robotContainer.targetHub = alliance == Alliance.Red ? m_robotContainer.RED_HUB : m_robotContainer.BLUE_HUB;
    m_objectField.setRobotPose(m_robotContainer.targetHub);
    SmartDashboard.putData("Object Field", this.m_objectField);
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
    }
    if (poseEstimate != null && poseEstimate.tagCount > 0 && Math.abs(omegarps) < 1.0) {
      m_robotContainer.drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
            VecBuilder.fill(.9, .9, 999999));
    }

    m_botField.setRobotPose(botState.Pose);
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
  public void disabledPeriodic() {}

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
