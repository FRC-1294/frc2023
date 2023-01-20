/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

    // Encoders
    public static final double kAngularEncoderConversionFactor = 2 * Math.PI * 1.0/18;

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final Boolean kTuningPID = true;
    
    // SPARK ids
    public static final int kFrontLeftSteerMotorSparkID = 14;
    public static final int kFrontLeftDriveMotorSparkID = 13;
    
    public static final int kFrontRightSteerMotorSparkID = 2;
    public static final int kFrontRightDriveMotorSparkID = 4;

    public static final int kBackLeftSteerMotorSparkID = 3;
    public static final int kBackLeftDriveMotorSparkID = 12;

    public static final int kBackRightSteerMotorSparkID = 1;
    public static final int kBackRightDriveMotorSparkID = 5;

    // Joystick ports
    public static int kRotationJoystickPort = 0;
    public static int kDrivingJoystickPort = 1;

    // TEJA IS COOL
    // TODO: Intakes, please correct these values
    public static final int kIntakeSolenoid1ChannelID = 13;
    public static final int kCompressorModuleID = 1;
}
