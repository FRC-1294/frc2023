package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveModuleEnum;
public class SwerveModule {
    private final CANSparkMax _drivingMotor;
    private final CANSparkMax _steeringMotor;

    private final RelativeEncoder _drivingEncoder;
    private final AnalogEncoder  _steeringEncoder;
    
    public final SwerveModuleEnum _swerveModuleData;

    public SwerveModule(SwerveModuleEnum swerveModuleInfo) {
        _swerveModuleData = swerveModuleInfo;

        _drivingMotor = new CANSparkMax(swerveModuleInfo.drivingMotorCANId, MotorType.kBrushless);
        _drivingMotor.setInverted(swerveModuleInfo.drivingMotorInverted);

        _drivingEncoder = _drivingMotor.getEncoder();        
        _drivingEncoder.setPositionConversionFactor(Constants.kDriveMotorPositionConversionFactor);
        _drivingEncoder.setVelocityConversionFactor(Constants.kDriveMotorVelocityConversionFactor);
        
        _steeringMotor = new CANSparkMax(swerveModuleInfo.steeringMotorCANId, MotorType.kBrushless);
        _steeringMotor.setInverted(swerveModuleInfo.steeringMotorInverted);
        
        _steeringEncoder = new AnalogEncoder(swerveModuleInfo.absEncoderAnalogPortNum);
        printOutAbsEncoderOffset(_swerveModuleData);        
        _steeringEncoder.setPositionOffset(swerveModuleInfo.absEncoderOffsetRotation);

        // _swerveModuleData.steeringMotorPositionController.enableContinuousInput(0, 2*Math.PI);
        _swerveModuleData.steeringMotorPositionController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();

        // TODO: current value is 5 degree. Might requires a better position tolerance value
        _swerveModuleData.steeringMotorPositionController.setTolerance(Constants.kSteeringPidControllerTolerance);

        setDrivingIdleMode(swerveModuleInfo.drivingMotorIdleMode);
        setSteeringIdleMode(swerveModuleInfo.steeringMotorIdleMode);
        burnSparks();
    }

    public void setDrivingIdleMode(CANSparkMax.IdleMode idleMode) {
        _drivingMotor.setIdleMode(idleMode);
    }

    public void setSteeringIdleMode(CANSparkMax.IdleMode idleMode) {
        _steeringMotor.setIdleMode(idleMode);
    }

    public void burnSparks(){
        REVLibError driveFlashState = _drivingMotor.burnFlash();
        REVLibError steeringFlashState = _steeringMotor.burnFlash();
        if (driveFlashState != REVLibError.kOk || steeringFlashState != REVLibError.kOk) {
            throw new RuntimeException(
                "BurnSparks failed. Error codes = (" + driveFlashState.value + ", " +steeringFlashState.value + ")");
        }
    }

    public void stop() {
        _drivingMotor.set(0);
        _steeringMotor.set(0);        
    }

    /**
     * Set the module's angle so that the wheels has X shape 
     * to ensure the robot do not skid.
     */
    public void lockModule() {
        switch (_swerveModuleData) {
            case FRONT_LEFT:
                setDesiredAngleRadians(Rotation2d.fromDegrees(45).getRadians());
                break;
            case FRONT_RIGHT:
                setDesiredAngleRadians(Rotation2d.fromDegrees(-45).getRadians());
                break;
            case BACK_LEFT:
                setDesiredAngleRadians(Rotation2d.fromDegrees(-45).getRadians());
                break;
            case BACK_RIGHT:
                setDesiredAngleRadians(Rotation2d.fromDegrees(45).getRadians());
                break;
            default:
                throw new RuntimeException("Invalid _swerveModuleData");
        }
    }

    public SwerveModuleState getState() {
        double drivingMotorSpeed = getDrivingSpeedMetersPerSecond();
        Rotation2d angle = Rotation2d.fromRadians(getSteeringEncoderValueRadians());
        return new SwerveModuleState(drivingMotorSpeed, angle);
    }

    public SwerveModulePosition getPosition() {
        double meters = getDrivingEncoderPositionMeters();
        Rotation2d angle = Rotation2d.fromRadians(getSteeringEncoderValueRadians());
        return new SwerveModulePosition(meters, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setDesiredDrivingSpeedMPS(desiredState.speedMetersPerSecond);
        setDesiredAngleRadians(desiredState.angle.getRadians());
    }

    public void setDesiredDrivingSpeedMPS(double desiredSpeedMPS) {

        // Temporarily disable PID controller approach because PID tuning is not good yet 
        // double driveSpeedPercent = _swerveModuleData.drivingMotorSpeedController.calculate(getDrivingSpeedMetersPerSecond(), desiredSpeedMPS);        
        if (Math.abs(desiredSpeedMPS) < 0.001) {
            _drivingMotor.set(0);
            return;
        }

        double driveSpeedPercent = desiredSpeedMPS / Constants.kDrivingMotorMaxSpeedMetersPerSecond;
        _drivingMotor.set(driveSpeedPercent);
    }

    public void setDesiredAngleRadians(double desiredAngleRadians) {
        double steeringSpeedPercent = _swerveModuleData.steeringMotorPositionController.calculate(
            getSteeringEncoderValueRadians(), 
            desiredAngleRadians);
        
        if (_swerveModuleData.steeringMotorPositionController.atSetpoint()) {
            _steeringMotor.set(0);
            return;
        }

        _steeringMotor.set(steeringSpeedPercent);
    }

    private void printOutAbsEncoderOffset(SwerveModuleEnum swerveModuleInfo) {
        String dashboardKey = swerveModuleInfo.name + "_AbsEncoderOffset";
        double absEncoderPosition = _steeringEncoder.getAbsolutePosition();
        SmartDashboard.putNumber(dashboardKey, absEncoderPosition);
        System.out.println(dashboardKey +  + absEncoderPosition);
    }

    public void resetEncoders() {
        _drivingEncoder.setPosition(0);
        _steeringEncoder.reset();
        _steeringEncoder.setPositionOffset(_swerveModuleData.absEncoderOffsetRotation);
        _swerveModuleData.steeringMotorPositionController.reset(
            getDrivingEncoderPositionMeters(),
            getDrivingSpeedMetersPerSecond());
    }

    public double getDrivingEncoderPositionMeters() {
        return _drivingEncoder.getPosition();
    }
    
    public double getDrivingSpeedMetersPerSecond() {
        return _drivingEncoder.getVelocity();
    }

    public double getDrivingSpeedRPM() {
        return getDrivingSpeedMetersPerSecond() / Constants.kDriveMotorVelocityConversionFactor;
    }

    public double getSteeringEncoderValueRadians() {
        double rotations = _steeringEncoder.get();
        double position = Math.IEEEremainder(rotations, 1.0);
        // SmartDashboard.putNumber("steeringEncoder_get", rotations);        
        return position * 2 * Math.PI;
    }
}