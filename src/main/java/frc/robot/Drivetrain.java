package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleEnum;

public class Drivetrain extends SubsystemBase {
    
    private final SwerveModule _frontLeft = new SwerveModule(SwerveModuleEnum.FRONT_LEFT);
    private final SwerveModule _frontRight = new SwerveModule(SwerveModuleEnum.FRONT_RIGHT);
    private final SwerveModule _backLeft = new SwerveModule(SwerveModuleEnum.BACK_LEFT);
    private final SwerveModule _backRight = new SwerveModule(SwerveModuleEnum.BACK_RIGHT);

    private final AHRS _navx; 
    private final SwerveDriveOdometry _odometry; 
    
    public volatile boolean DRIVE_MODE_FIELD_ORIENTED = true;

    public Drivetrain() {
        _navx = new AHRS(SPI.Port.kMXP);
        _odometry = new SwerveDriveOdometry(
            Constants.kDriveKinematics, 
            new Rotation2d(0), 
            getModulePositions());        
    }

    public SwerveModule getModule(SwerveModuleEnum whichModule) {
        switch (whichModule) {
            case FRONT_LEFT:
                return _frontLeft;
            case FRONT_RIGHT:
                return _frontRight;
            case BACK_LEFT:
                return _backLeft;
            case BACK_RIGHT:
                return _backRight;
            default:
                throw new RuntimeException("Invalid module " + whichModule.name);
        }
    }

    /**
     * Method to drive the robot using (x, y, theta) speeds
     * @param xSpeed Speed of the robot in the x direction (forwards)
     * @param ySpeed Speed of the robot in the y direction (sideways)
     * @param thetaSpeed Angular speed of the robot
     */
    public void drive(double xSpeed, double ySpeed, double thetaSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        if(DRIVE_MODE_FIELD_ORIENTED) {
            ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroAngleRotation2d());
        }
        SwerveModuleState[] desiredStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(desiredStates);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            _frontLeft.getPosition(),
            _frontRight.getPosition(), 
            _backLeft.getPosition(), 
            _backRight.getPosition()};
    }

    @Override
    public void periodic() {
        // TODO: Remove this line 
        NetworkTableInstance.getDefault().flush();        

        // TODO: Validate this
        _odometry.update(getGyroAngleRotation2d(), getModulePositions()); 

        SmartDashboard.putNumberArray(
            "RobotLocation(x, y, theta)",
            new double[] {
                getRobotPose().getX(),
                getRobotPose().getY(),
                getRobotPose().getRotation().getDegrees(),
            });
    }

    public void resetGyroHeading() {
        _navx.reset();
    }

    public void stopModules() {
        _frontLeft.stop();
        _frontRight.stop();
        _backLeft.stop();
        _backRight.stop();
    }

    public void resetEncoders() {
        _frontLeft.resetEncoders();
        _frontRight.resetEncoders();
        _backLeft.resetEncoders();
        _backRight.resetEncoders();
    }

    public void resetOdometry(Pose2d pose) {
        // TODO: Validate this
        _odometry.resetPosition(getGyroAngleRotation2d(), getModulePositions(), pose);
    }

    public Pose2d getRobotPose() {
        return _odometry.getPoseMeters();
    }

    public double getTurnRateDegreesPerSec() {
        return _navx.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
    }
    
    public double getGyroAngleDegrees() {
        return getGyroAngleRotation2d().getDegrees();
    }

    public double getGyroAngleRadians() {
        return getGyroAngleRotation2d().getRadians();
    }

    public Rotation2d getGyroAngleRotation2d() {
        return _navx.getRotation2d();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivingMotorMaxSpeedMetersPerSecond);
        _frontLeft.setDesiredState(states[0]);
        _frontRight.setDesiredState(states[1]);
        _backLeft.setDesiredState(states[2]);
        _backRight.setDesiredState(states[3]);
    } 

    public void setModuleDrivingSpeed(double desiredSpeedRPM, SwerveModuleEnum whichModule) {
        switch (whichModule) {
            case FRONT_LEFT:
                _frontLeft.setDesiredDrivingSpeedMPS(desiredSpeedRPM);
                break;
            case FRONT_RIGHT:
                _frontRight.setDesiredDrivingSpeedMPS(desiredSpeedRPM);
                break;
            case BACK_LEFT:
                _backLeft.setDesiredDrivingSpeedMPS(desiredSpeedRPM);
                break;
            case BACK_RIGHT:
                _backRight.setDesiredDrivingSpeedMPS(desiredSpeedRPM);
                break;
            default:
                throw new RuntimeException("Invalid module " + whichModule.name);
        }
    } 

    public void setModuleSteeringAngle(double desiredAngleRadians, SwerveModuleEnum whichModule) {
        switch (whichModule) {
            case FRONT_LEFT:
                _frontLeft.setDesiredAngleRadians(desiredAngleRadians);
                break;
            case FRONT_RIGHT:
                _frontRight.setDesiredAngleRadians(desiredAngleRadians);
                break;
            case BACK_LEFT:
                _backLeft.setDesiredAngleRadians(desiredAngleRadians);
                break;
            case BACK_RIGHT:
                _backRight.setDesiredAngleRadians(desiredAngleRadians);
                break;
            default:
                throw new RuntimeException("Invalid module " + whichModule.name);
        }
    } 

    public void lockModules() {
        _frontLeft.lockModule();
        _frontRight.lockModule();
        _backLeft.lockModule();
        _backRight.lockModule();
    }
}