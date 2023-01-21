/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Input  
  public static final int kXboxControllerPort = 0;
  public static final int kXboxControllerRobotOrientedButtonIdx = 1;

  public static final int kXboxControllerYAxis = 1;
  public static final int kXboxControllerXAxis = 0;
  public static final int kXboxControllerRotAxis = 4;
  
  public static final double kDeadband = 0.05;  

  // SWERVE constants 
  public static final double kDriveMotorMaxSpeedMeterPerSecond = 12.0;

  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

  public static final double kTeleDriveMaxSpeedMetersPerSecond = 1;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
  public static final double kDriveEncoderRot2Meter = Math.PI * Units.inchesToMeters(4);
  public static final double kDriveGearRation = 1/10;
  public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter*kDriveGearRation / 60;

  // Distance between right and left wheels
  // public static final double kTrackWidth = Units.inchesToMeters(25.5);
  public static final double kTrackWidth = Units.inchesToMeters(18.5);

  // Distance between front and back wheels
  public static final double kWheelBase = Units.inchesToMeters(18.5);

  public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
    new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
    new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
    new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2));

  // Encoders
  public static final double kAngularEncoderConversionFactor = 2 * Math.PI * 1.0/18;

  // Use PIDtunning.java to find Global.kP and Global.kD values
  public static final boolean kTuningPID = true;
  
  public static final Gains kSwerveModuleSteeringMotorPIDConstants = new Gains(0.1, 0, 0);

  // SPARK ids
  public static final int kFrontLeftDriveMotorSparkID = 2;
  public static final int kFrontLeftSteerMotorSparkID = 3;
  
  public static final int kFrontRightDriveMotorSparkID = 4;
  public static final int kFrontRightSteerMotorSparkID = 5;

  public static final int kBackLeftDriveMotorSparkID = 6;
  public static final int kBackLeftSteerMotorSparkID = 7;

  public static final int kBackRightDriveMotorSparkID = 8;
  public static final int kBackRightSteerMotorSparkID = 9;

  // Joystick ports
  public static int kRotationJoystickPort = 0;
  public static int kDrivingJoystickPort = 1;

  // TEJA IS COOL
  // TODO: Intakes, please correct these values
  public static final int kIntakeSolenoid1ChannelID = 13;
  public static final int kCompressorModuleID = 1;
}
