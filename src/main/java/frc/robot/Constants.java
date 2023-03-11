package frc.robot;
import java.util.Map;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {

    public static String getPoseString(Pose2d pose) {
        return String.format(
            "(%.2f, %.2f, %.2f degrees)",
            pose.getX(), 
            pose.getY(),
            pose.getRotation().getDegrees());
    }
    
    public static final boolean kGyroReversed = true;

    public static final XboxController kXboxController = new XboxController(0);
    
    public static final double kNeoMotorFreeSpeedRPM = 5676;

    public static final double kWheelRadiusMeters = Units.inchesToMeters(2);
    public static final double kDriveMotorGearReduction = 1.0/10;
    public static final double kSteeringMotorGearReduction = 1.0/18;

    public static final double kDriveMotorPositionConversionFactor = 
        kDriveMotorGearReduction * 2 * Math.PI * kWheelRadiusMeters;

    public static final double kDriveMotorVelocityConversionFactor = 
        (kDriveMotorGearReduction / 60) * 2 * Math.PI * kWheelRadiusMeters;
    
    // location for the swerve drive module relative to the robot center
    public static final double kTrackWidth = 0.54;
    public static final double kWheelBase = 0.54;

    public static final Translation2d kFrontLeftLocation = new Translation2d(kWheelBase/2.0, kTrackWidth/2.0);
    public static final Translation2d kFrontRightLocation = new Translation2d(kWheelBase/2.0, -kTrackWidth/2.0);
    public static final Translation2d kBackLeftLocation = new Translation2d(-kWheelBase/2.0, kTrackWidth/2.0);
    public static final Translation2d kBackRightLocation = new Translation2d(-kWheelBase/2.0, -kTrackWidth/2.0);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation,
        kFrontRightLocation,
        kBackLeftLocation,
        kBackRightLocation);

    // public static final double kDrivingMotorMaxSpeedMetersPerSecond = 4.0;
     public static final double kDrivingMotorMaxSpeedMetersPerSecond = /* 3.019497 */
            (kNeoMotorFreeSpeedRPM / 60) * kDriveMotorGearReduction * 2 * Math.PI * kWheelRadiusMeters;
    
    public static final double kDrivingMotorMaxAccelerationMetersPerSecondSq = 3;

    public static final double kSteeringMotorMaxSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kSteeringMotorMaxAccelerationRadiansPerSecondSquared = Math.PI;

    public static final TrapezoidProfile.Constraints kSteeringProfileConstraints = new TrapezoidProfile.Constraints(
        kSteeringMotorMaxSpeedRadiansPerSecond, 
        kSteeringMotorMaxAccelerationRadiansPerSecondSquared);

    public static final double kSteeringPidControllerTolerance = Units.degreesToRadians(5);

    public enum SwerveModuleEnum { 
        FRONT_LEFT (
            "FrontLeft" /* name of the module */, 
            1 /* drive motor CAN ID */, 
            3 /* rotation motor CAN ID */, 
            false /* drive motor inversion */, 
            false /* rotation motor inversion */,
            IdleMode.kBrake /* driving motor idle mode; */,
            IdleMode.kBrake /* steering motor idle mode; */,
            0 /* Absolute encoder analog input port number */,
            false /* Absolute encoder inversion */,
            0.889 /* Absolute encoder offset position, values from 0 to 1.0 */,
            new PIDController(0.5, 0.0, 0.0) /* velocity controller parameters for driving motor */,
            new ProfiledPIDController(0.56, 0.025, 0.01, kSteeringProfileConstraints) /* position controller parameters for steering motor */),

        FRONT_RIGHT (
            "FrontRight" /* name of the module */, 
            7 /* drive motor CAN ID */, 
            6 /* rotation motor CAN ID */, 
            true /* drive motor inversion */, 
            false /* rotation motor inversion */,
            IdleMode.kBrake /* driving motor idle mode; */,
            IdleMode.kBrake /* steering motor idle mode; */,
            1 /* Absolute encoder analog input port number */,
            false /* Absolute encoder inversion */,
            0.828 /* Absolute encoder offset position, values from 0 to 1.0 */,
            new PIDController(0.5, 0.0, 0.0) /* velocity controller parameters for driving motor */,
            new ProfiledPIDController(0.771, 0.025, 0.015, kSteeringProfileConstraints) /* position controller parameters for steering motor */),

        BACK_LEFT (
            "BackLeft" /* name of the module */, 
            4 /* drive motor CAN ID */, 
            5 /* rotation motor CAN ID */, 
            false /* drive motor inversion */, 
            false /* rotation motor inversion */,
            IdleMode.kBrake /* driving motor idle mode; */,
            IdleMode.kBrake /* steering motor idle mode; */,
            2 /* Absolute encoder analog input port number */,
            false /* Absolute encoder inversion */,
            0.752 /* Absolute encoder offset position, values from 0 to 1.0 */,
            new PIDController(0.5, 0.0, 0.0) /* velocity controller parameters for driving motor */,
            new ProfiledPIDController(0.56, 0.0, 0.01, kSteeringProfileConstraints) /* position controller parameters for steering motor */),

        BACK_RIGHT (
            "BackRight" /* name of the module */, 
            8 /* drive motor CAN ID */, 
            9 /* rotation motor CAN ID */, 
            true /* drive motor inversion */, 
            false /* rotation motor inversion */,
            IdleMode.kBrake /* driving motor idle mode; */,
            IdleMode.kBrake /* steering motor idle mode; */,
            3 /* Absolute encoder analog input port number */,
            false /* Absolute encoder inversion */,
            0.583 /* Absolute encoder offset position, values from 0 to 1.0 */,
            new PIDController(0.5, 0.0, 0.0) /* velocity controller parameters for driving motor */,
            new ProfiledPIDController(0.56, 0.0, 0.015, kSteeringProfileConstraints) /* position controller parameters for steering motor */);
        
        public final String name; 
        public final int drivingMotorCANId; 
        public final int steeringMotorCANId;
        public final boolean drivingMotorInverted;
        public final boolean steeringMotorInverted;
        public final IdleMode drivingMotorIdleMode;
        public final IdleMode steeringMotorIdleMode;

        public final int absEncoderAnalogPortNum;
        public final boolean absEncoderInverted;
        public final double absEncoderOffsetRotation;
        public final PIDController drivingMotorSpeedController;
        public final ProfiledPIDController steeringMotorPositionController;

        private SwerveModuleEnum(
            String name,
            int drivingMotorCANId, 
            int steeringMotorCANId,
            boolean drivingMotorInverted,
            boolean steeringMotorInverted,
            IdleMode drivingMotorIdleMode,
            IdleMode steeringMotorIdleMode,
            int absEncoderAnalogPortNum,
            boolean absEncoderInverted,
            double absEncoderOffsetRotation,
            PIDController drivingMotorSpeedController,
            ProfiledPIDController steeringMotorPositionController) {

            this.name = name;
            this.drivingMotorCANId = drivingMotorCANId;
            this.steeringMotorCANId = steeringMotorCANId;

            this.drivingMotorInverted = drivingMotorInverted;
            this.steeringMotorInverted = steeringMotorInverted;

            this.drivingMotorIdleMode = drivingMotorIdleMode;
            this.steeringMotorIdleMode = steeringMotorIdleMode;

            this.absEncoderAnalogPortNum = absEncoderAnalogPortNum;
            this.absEncoderInverted = absEncoderInverted;
            this.absEncoderOffsetRotation = absEncoderOffsetRotation;

            this.drivingMotorSpeedController = drivingMotorSpeedController;
            this.steeringMotorPositionController = steeringMotorPositionController;
        }
    }

    public static final class VisionConstants {

        public final static double kMaxMeasurementMeters = 2.0;

        public final static boolean kFastVisionSystem = false;
        public final static double kDetectionAmbiguityTolerance = 0.2;

        public static final Map<Integer, Pose3d> kTargetPoseMap = Map.of(
            -1, new Pose3d(0, 1, 0.5, new Rotation3d(0, 0, Units.degreesToRadians(180))), 
            -2, new Pose3d(0, 2, 0.5, new Rotation3d(0, 0, Units.degreesToRadians(180)))
        );
    
        public static final Transform3d kCameraToRobot = new Transform3d(
            new Translation3d(0.4, 0.0, 0.3),
            new Rotation3d(0, 0, 0)
        );

        public static final Transform3d kRobotToCamera = kCameraToRobot.inverse();
        
        public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs =  VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));                

        public static final TrapezoidProfile.Constraints kXControllerConstraints = new TrapezoidProfile.Constraints(3, 2);
        public static final TrapezoidProfile.Constraints kYControllerConstraints = new TrapezoidProfile.Constraints(3, 2);
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(8, 8);        

        public static final int kTagToChase = 2;
        public static final Transform3d kTagToGoal = new Transform3d(
            new Translation3d(1.5, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI)
        );

        public static final ProfiledPIDController xController = 
            new ProfiledPIDController(2.6,  0, 0, kXControllerConstraints);

        public static final ProfiledPIDController yController = 
            new ProfiledPIDController(2.6,  0, 0, kYControllerConstraints);

        public static final ProfiledPIDController omegaController = 
            new ProfiledPIDController(2.6,  0, 0, kThetaControllerConstraints);
    }

    public static final class AutoConstants {
        public static final double kDrivingMaxSpeedMetersPerSecond = kDrivingMotorMaxSpeedMetersPerSecond;
        public static final double kDrivingMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSq = Math.PI;

        public static final TrapezoidProfile.Constraints kHeadingControllerConstraints = new TrapezoidProfile.Constraints(
            kSteeringMotorMaxSpeedRadiansPerSecond, 
            kSteeringMotorMaxAccelerationRadiansPerSecondSquared);
    
        public static final double kPositionToleranceMeters = 0.05;
        public static final double kAngleToleranceRadians = Units.degreesToRadians(4);

        public static final PIDController xPositionController  = 
            new PIDController(3.0, 0, 0);

        public static final PIDController yPositionController  = 
            new PIDController(3.0, 0, 0);

        public static final ProfiledPIDController omegaController = 
            new ProfiledPIDController(2.6,  0, 0, kHeadingControllerConstraints);
        
        public static void debugPositionControllers(String desc) {
            SmartDashboard.putNumberArray(desc,
            new double[] {
                xPositionController.getP(),
                xPositionController.getI(),
                xPositionController.getD(),
                yPositionController.getP(),
                yPositionController.getI(),
                yPositionController.getD(),
                omegaController.getP(),
                omegaController.getI(),
                omegaController.getD(),
            });
        }
    }
}