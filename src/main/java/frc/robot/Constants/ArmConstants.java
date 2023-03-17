package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final double zeroAngleRad = Units.degreesToRadians(6.5);
    public static final double minAngleRad = Units.degreesToRadians(27.0);
    public static final double maxAngleRad = Units.degreesToRadians(115.0);

    public static final double kArmExtensionMinPositionInches = 41;
    public static final double kArmExtensionMinTravelInches = 0;
    public static final double kArmExtensionMaxTravelInches = 18.3;

    public static final double kArmExtensionMaxPositionInches = 
        kArmExtensionMaxTravelInches + 
            (kArmExtensionMinPositionInches - kArmExtensionMinTravelInches) ;

    public enum ArmMode {
        REST (
            "Neutral" /* name of the arm mode */,
            1 /* arm pivot angle in degrees */,
            3 /* extension travel in inches, from MinTravelInches to MaxTravelInches */),
        
        OFF_GROUND (
            "Neutral" /* name of the arm mode */,
            1 /* arm pivot angle in degrees */,
            3 /* extension travel in inches, from MinTravelInches to MaxTravelInches */),

        SUB_STATION (
            "Neutral" /* name of the arm mode */,
            1 /* arm pivot angle in degrees */,
            3 /* extension travel in inches, from MinTravelInches to MaxTravelInches */),
        
        NODE1 (
            "Neutral" /* name of the arm mode */,
            1 /* arm pivot angle in degrees */,
            3 /* extension travel in inches, from MinTravelInches to MaxTravelInches */),
        
        NODE2 (
            "Neutral" /* name of the arm mode */,
            1 /* arm pivot angle in degrees */,
            3 /* extension travel in inches, from MinTravelInches to MaxTravelInches */),
        
        NODE3 (
            "Neutral" /* name of the arm mode */,
            1 /* arm pivot angle in degrees */,
            3 /* extension travel in inches, from MinTravelInches to MaxTravelInches */)
        
        public final String name;
        public final double pivotAngleDegrees;
        public final double extensionTravelInches;

        private ArmMode(String name, double pivotAngleDegrees, double extensionTravelInches) {
            this.name = name;
            this.pivotAngleDegrees = pivotAngleDegrees;
            this.extensionTravelInches = extensionTravelInches;
        }
    }

    // Setpoints
    public static final double[] extensionLevelsIn = {kArmExtensionMinPositionInches, kArmExtensionMinPositionInches, 43.0}; //inches
    public static final double[] angleLevelsDeg = {35.0, 85.0, 99.75}; //degrees

    public static final double[] offSubstation = {85.5, 46.0}; // angle, inches including claw
//    public static final double[] offGround


    //CAP af find this!!!
    public static final double pivotPosInMetersY = Units.inchesToMeters(45.75);


    // Device IDs
    public static final int kArmPivotLeftMotorCANId = 15;
    public static final int kArmPivotRightMotorCANId = 16;

    public static final int telescopicArmSpark = 10;

    public static final double relEncoderToInitialGear = 1.0/48;

    public static final double falconToFinalGear = 1.0/240;
    public static final double encoderResolution = 1.0/2048;

    public static boolean leftPivotInverted = true;
}