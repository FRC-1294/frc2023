package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.Constants.AutoConstants;

public class SwerveTrajectoryTuning extends CommandBase{

    private final int kTuneX = 0, kTuneY = 1, kTuneTheta = 2;        
    private final Drivetrain _drivetrain;

    private final Trigger _bButton;
    private volatile boolean _runMoveTo;

    private final MoveTo _moveForward;
    private final MoveTo _moveBackward;
    private final SequentialCommandGroup _moveForwardBackward;

    private final Trigger _xButton;
    private int _tuneId = kTuneX;

    public SwerveTrajectoryTuning(Drivetrain drivetrain) {

        _drivetrain = drivetrain;
        _drivetrain.resetEncoders();
        _drivetrain.resetGyroHeading();

        addRequirements(_drivetrain);
        
        _moveForward = new MoveTo(drivetrain, new Pose2d(3, 0, new Rotation2d()));
        _moveBackward = new MoveTo(drivetrain, new Pose2d(0, 0, new Rotation2d()));
        _moveForwardBackward = new SequentialCommandGroup(_moveForward, _moveBackward);

        _bButton = new JoystickButton(Constants.kXboxController, XboxController.Button.kB.value);
        _bButton.onTrue(
            new InstantCommand(
                () -> _runMoveTo = !_runMoveTo, 
                _drivetrain
            )
        );

        _xButton = new JoystickButton(Constants.kXboxController, XboxController.Button.kX.value);
        _xButton.onTrue(
            new InstantCommand(
                () -> updateTuneId(), 
                _drivetrain
            ) 
        );
    }

    private void updateTuneId() {
        _tuneId = (_tuneId + 1) % 3;
    }

    @Override
    public void initialize() {
        _runMoveTo = false;
    }
    
    @Override
    public void execute() {
        SmartDashboard.putBoolean("_runMoveTo", _runMoveTo);
        SmartDashboard.putNumber("tuneId", _tuneId);
        SmartDashboard.putBooleanArray(
            "Forward_Backward", 
            new boolean[] {
                _moveForward.isScheduled(),
                _moveForward.isFinished(),
                _moveBackward.isScheduled(),
                _moveBackward.isFinished(),
            });

        if (_runMoveTo) {            
            _moveForwardBackward.repeatedly().schedule();
        } else {
            AutoConstants.debugPositionControllers("SwerveTrajectoryTuning_execute_debug");

            if (Constants.kXboxController.getYButtonPressed()) {
                switch (_tuneId) {
                    case kTuneX:
                        AutoConstants.xPositionController.setP(AutoConstants.xPositionController.getP() + 0.1);
                        break;
                    case kTuneY:
                        AutoConstants.yPositionController.setP(AutoConstants.yPositionController.getP() + 0.1);
                        break;
                    case kTuneTheta:
                        AutoConstants.omegaController.setP(AutoConstants.omegaController.getP() + 0.1);
                        break;                
                    default:
                        break;
                } 
            } else if (Constants.kXboxController.getAButtonPressed()) {
                switch (_tuneId) {
                    case kTuneX:
                        AutoConstants.xPositionController.setP(AutoConstants.xPositionController.getP() - 0.05);
                        break;
                    case kTuneY:
                        AutoConstants.yPositionController.setP(AutoConstants.yPositionController.getP() - 0.05);
                        break;
                    case kTuneTheta:
                        AutoConstants.omegaController.setP(AutoConstants.omegaController.getP() - 0.05);
                        break;                
                    default:
                        break;
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
}
