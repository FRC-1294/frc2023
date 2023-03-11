package frc.robot.command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveModuleEnum;

public class SteeringMotorPidTuning extends CommandBase {

    private final Drivetrain _drivetrain;

    private double _kP;
    private double _measuredAngleRadians;

    private double[] _desiredAngleRadiansSetPoints;
    private int _desiredAngleRadiansSetPointIndex;
    private double _desiredAngleRadiansSetPoint;

    private SwerveModuleEnum _swerveModuleEnum;
    private SwerveModule _swerveModule;

    private double _errorRadians;

    public SteeringMotorPidTuning(
        Drivetrain drivetrain,
        SwerveModuleEnum swerveModuleEnum) {

        _drivetrain = drivetrain;
        _swerveModuleEnum = swerveModuleEnum;

        addRequirements(_drivetrain);

        _desiredAngleRadiansSetPoints = new double[] {
            Math.PI / 4,
            Math.PI / 2,
            3 * Math.PI / 4,
            Math.PI,
            5 * Math.PI / 4,
            3 * Math.PI / 2,
            7 * Math.PI / 4,
            2 * Math.PI
        };
        
        _desiredAngleRadiansSetPointIndex = 0;
        _desiredAngleRadiansSetPoint = _desiredAngleRadiansSetPoints[_desiredAngleRadiansSetPointIndex];

        _swerveModule = _drivetrain.getModule(_swerveModuleEnum);

        _drivetrain.resetEncoders();
        _measuredAngleRadians = _swerveModule.getSteeringEncoderValueRadians();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {        
        if (Constants.kXboxController.getYButtonPressed()) {
            _kP = _swerveModule._swerveModuleData.steeringMotorPositionController.getP() + 0.01;
            _swerveModule._swerveModuleData.steeringMotorPositionController.setP(_kP);
        } else if (Constants.kXboxController.getAButtonPressed()) {
            _kP = _swerveModule._swerveModuleData.steeringMotorPositionController.getP() - 0.005;
            _swerveModule._swerveModuleData.steeringMotorPositionController.setP(_kP);
        } else if (Constants.kXboxController.getXButtonPressed()) {
            _desiredAngleRadiansSetPointIndex = (_desiredAngleRadiansSetPointIndex + 1) % 8; 
            _desiredAngleRadiansSetPoint = _desiredAngleRadiansSetPoints[_desiredAngleRadiansSetPointIndex];            
        }

        _swerveModule.setDesiredAngleRadians(_desiredAngleRadiansSetPoint);
        _measuredAngleRadians = _swerveModule.getSteeringEncoderValueRadians();
        _errorRadians = _desiredAngleRadiansSetPoint - _measuredAngleRadians;

        SmartDashboard.putString("Module", _swerveModuleEnum.name);
        SmartDashboard.putNumber("MeasuredAngleRadians", _measuredAngleRadians);
        SmartDashboard.putNumber("DesiredAngleRadians", _desiredAngleRadiansSetPoint);
        SmartDashboard.putNumber("ErrorRadians", _errorRadians);
        SmartDashboard.putNumber("kP", _kP);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
