/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;

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
    //SWERVE constants

    public static final double maxSpeed = 12.0;
    
    public static final double rad2Deg = 180/Math.PI;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5; //40
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
    public static final double kDriveEncoderRot2Meter = Math.PI * Units.inchesToMeters(4);
    public static final double kDriveGearRation = 1/10;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter*kDriveGearRation / 60;

    //Encoders
    public static final double angleEncoderConversionFactor = 2*Math.PI/18;
    public static final double driveEncoderConversionFactor = 1;

    //arm constants
    public static final double minAngle = 5.0;
    public static final double maxAngle = 115.0;

    public static final double[] angleLevels = {42.0, 100.976, 107.414};




    //PID
    public static final Gains anglePID = new Gains(0.005, 0, 0, 0.0, 0.0, -0.5, 0.5, 0);
    public static final Gains anglePIDFast = new Gains(0.005, 0, 0, 0.0, 0.0, -1, 1, 1);
    public static final Gains fastPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -1, 1, 1);
    public static final int swervePIDSlot = anglePIDFast.kSlot;
    public static final double PIDdiff = 1;
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    public static double tuningSetpoint = 0;
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    public static final Boolean tuningPID = true;
    public static final Boolean fieldOriented = true;
    //JOYSTICK constants
    public static final double deadzone = 0.1;

    //SPARK ids
    public static final int frontLeftSteer = 3;//
    public static final int frontLeftDrive = 2;//
    
    public static final int frontRightSteer = 5;//
    public static final int frontRightDrive = 4;//

    public static final int rearLeftSteer = 7;//
    public static final int rearLeftDrive = 6;//

    public static final int rearRightSteer = 9;//
    public static final int rearRightDrive = 8; // 


    //arm stuff
    
    public static final int rightArmPivot = 15;
    public static final int leftArmPivot = 16;

    public static final int armPivotEncoderPort = 17;


    //used for offset because the absolute encoder will likely not be at zero in the initial rotation of the arm
    public static final double armPivotDegreesAtRest = 0.0;


    public static final double kRotP = 0.005;

    //TEJA IS COOL

    //Change this
    public static int rotJoystickPort = 0;
    public static int transJoystickPort = 1;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static int intakeSolenoid1ID;
    public static int compressorID;
}
