package frc.robot.Constants;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final double zeroAngleRad = Units.degreesToRadians(6.5);
    public static final double minAngleRad = Units.degreesToRadians(27.0);
    public static final double maxAngleRad = Units.degreesToRadians(115.0);

    public static final double kArmExtensionMinPositionInches = 41;
    public static final double zeroExtensionIn = 1.618 + kArmExtensionMinPositionInches;
    public static final double kArmExtensionMaxTravelInches = 16;
    public static final double kArmExtensionMaxPositionInches = kArmExtensionMaxTravelInches + kArmExtensionMinPositionInches;

    public static final double extensionEncoderToInches =  1.0/.25;

    public static final double maxExtensionIn = 57;

    // Setpoints
    public static final double[] extensionLevelsIn = {kArmExtensionMinPositionInches, kArmExtensionMinPositionInches, 43.0}; //inches
    public static final double[] angleLevelsDeg = {35.0, 85.0, 99.75}; //degrees

    public static final double[] offSubstation = {85.5, 46.0}; // angle, inches including claw
//    public static final double[] offGround



    //CAP af find this!!!
    public static final double pivotPosInMetersY = Units.inchesToMeters(45.75);


    //TBD
    public static final int rightArmPivot = 16;
    public static final int leftArmPivot = 15;
    public static final int telescopicArmSpark = 10;
    public static final int armPivotEncoderPort = 17;


    public static final double relEncoderToInitialGear = 1.0/48;

    public static final double falconToFinalGear = 1.0/240;
    public static final double encoderResolution = 1.0/2048;
    //Change this
    public static boolean leftPivotInverted = true;
}