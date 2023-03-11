package frc.robot.command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveModuleEnum;

public class DrivingMotorPidTuning extends CommandBase {

    private final Drivetrain _drivetrain;

    private double _kP;
    private double _measuredSpeedRPM;

    private double[] _desiredSpeedRPMSetPoints;
    private int _desiredSpeedSetPointIndex;
    private double _desiredSpeedRPMSetPoint;

    private SwerveModuleEnum  _swerveModuleEnum;
    private SwerveModule _swerveModule;

    private double _speedErrorRPM;

    public DrivingMotorPidTuning(
        Drivetrain drivetrain,
        SwerveModuleEnum swerveModuleEnum) {

        _drivetrain = drivetrain;
        _swerveModuleEnum = swerveModuleEnum;

        addRequirements(_drivetrain);

        _desiredSpeedRPMSetPoints = new double[] {
            Constants.kNeoMotorFreeSpeedRPM / 4,
            Constants.kNeoMotorFreeSpeedRPM / 2,
            3 * Constants.kNeoMotorFreeSpeedRPM / 4,
            0.95 * Constants.kNeoMotorFreeSpeedRPM,
            -Constants.kNeoMotorFreeSpeedRPM / 4,
            -Constants.kNeoMotorFreeSpeedRPM / 2,
            -3 * Constants.kNeoMotorFreeSpeedRPM / 4,
            -0.95 * Constants.kNeoMotorFreeSpeedRPM
        };

        _desiredSpeedSetPointIndex = 0;
        _desiredSpeedRPMSetPoint = _desiredSpeedRPMSetPoints[_desiredSpeedSetPointIndex];

        
        _swerveModule = _drivetrain.getModule(_swerveModuleEnum);

        _drivetrain.resetEncoders();
        _measuredSpeedRPM = _swerveModule.getDrivingSpeedMetersPerSecond();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {        
        if (Constants.kXboxController.getYButtonPressed()) {
            _kP = _swerveModule._swerveModuleData.drivingMotorSpeedController.getP();
            _kP += 0.01;            
            _swerveModule._swerveModuleData.drivingMotorSpeedController.setP(_kP);
        } else if (Constants.kXboxController.getAButtonPressed()) {
            _kP = _swerveModule._swerveModuleData.drivingMotorSpeedController.getP();
            _kP -= 0.005;
            _swerveModule._swerveModuleData.drivingMotorSpeedController.setP(_kP);
        } else if (Constants.kXboxController.getXButtonPressed()) {
            _desiredSpeedSetPointIndex = (_desiredSpeedSetPointIndex + 1) % 8; 
            _desiredSpeedRPMSetPoint = _desiredSpeedRPMSetPoints[_desiredSpeedSetPointIndex];            
        }

        var desiredDrivingSpeedMPS = _desiredSpeedRPMSetPoint * Constants.kDriveMotorVelocityConversionFactor;                
        _swerveModule.setDesiredDrivingSpeedMPS(desiredDrivingSpeedMPS);

        _measuredSpeedRPM = _swerveModule.getDrivingSpeedRPM();
        _speedErrorRPM = _desiredSpeedRPMSetPoint - _measuredSpeedRPM;

        SmartDashboard.putString("Module", _swerveModuleEnum.name);
        SmartDashboard.putNumber("MeasuredSpeedRPM", _measuredSpeedRPM);
        SmartDashboard.putNumber("DesiredSpeedRPM", _desiredSpeedRPMSetPoint);
        SmartDashboard.putNumber("SpeedErrorRPM", _speedErrorRPM);
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
