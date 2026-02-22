// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // Limelight names
  public static final String SHOOTER_LIMELIGHT_NAME = "limelight-shooter";
  public static final String FEEDER_LIMELIGHT_NAME = "limelight-intake";

  // left shooter IDs and channel
  public static final int JOHN_SHOOTER_MOTOR_ID = 3;
  public static final int JOHN_FEEDER_MOTOR_ID = 4;
  public static final int JOHN_LINEAR_ACTUATOR_CHANNEL = 3;
  public static final int JOHN_BEAMBREAK_CHANNEL = 2;

  // middle shooter IDs and channel
  public static final int JAWBREAKER_SHOOTER_MOTOR_ID = 1;
  public static final int JAWBREAKER_FEEDER_MOTOR_ID = 2;
  public static final int JAWBREAKER_LINEAR_ACTUATOR_CHANNEL = 2;
  public static final int JAWBREAKER_BEAMBREAK_CHANNEL = 0;

  // right shooter IDs and channel
  public static final int TAYLOR_SHOOTER_MOTOR_ID = 5;
  public static final int TAYLOR_FEEDER_MOTOR_ID = 6;
  public static final int TAYLOR_LINEAR_ACTUATOR_CHANNEL = 1;
  public static final int TAYLOR_BEAMBREAK_CHANNEL = 4;

  // intake motor IDs
  public static final int INTAKE_MOTOR_ID = 7;
  public static final int TILT_MOTOR_ID = 8;
  public static final int FLOOR_FEED_MOTOR_ID = 9;
  
  // climb motor ID
  public static final int CLIMB_MOTOR_ID = 10;

  // left front drive motor IDs
  public static final int LEFT_FRONT_DRIVE = 11;
  public static final int LEFT_FRONT_STEER = 13;
  
  // left rear drive motor IDs
  public static final int LEFT_REAR_STEER = 12;
  public static final int LEFT_REAR_DRIVE = 14;

  // right rear drive motor IDs
  public static final int RIGHT_REAR_STEER = 15;
  public static final int RIGHT_REAR_DRIVE = 17;
  
  // right front drive motor IDs
  public static final int RIGHT_FRONT_STEER = 16;
  public static final int RIGHT_FRONT_DRIVE = 18;

  // encoder IDs
  public static final int FRONT_LEFT_ENC = 19;
  public static final int BACK_LEFT_ENC = 20;
  public static final int FRONT_RIGHT_ENC = 21;
  public static final int BACK_RIGHT_ENC = 22;

  // pigeon ID
  public static final int PIGEON = 23;

}
