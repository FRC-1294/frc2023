package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.DriveForwardInches;
import frc.robot.command.SteeringMotorPidTuning;
import frc.robot.command.SwerveTrajectoryTuning;
import frc.robot.command.TagChaser;
import frc.robot.command.XBoxDriveCmd;

public class Robot extends TimedRobot {
    private final Drivetrain _drivetrain = new Drivetrain();
    private final SendableChooser<Command> _autoSelector = new SendableChooser<>();
    
    private Command _autoCommand;
    private final Trigger _leftBumperButton;
    private final Trigger _rightBumperButton;

    private final NetworkTableInstance _netTableInstance =  NetworkTableInstance.getDefault();
    private final PhotonCamera _camera = new PhotonCamera(_netTableInstance, "OV5647");

    public Robot() {        
        /* super(0.01); */ 
        _leftBumperButton = new JoystickButton(
            Constants.kXboxController,
            XboxController.Button.kLeftBumper.value);

        _rightBumperButton = new JoystickButton(
            Constants.kXboxController,
            XboxController.Button.kRightBumper.value);
    
        _leftBumperButton.onTrue(
            new InstantCommand(
                () -> _drivetrain.resetGyroHeading(), 
                _drivetrain));

        _rightBumperButton.onTrue(
            new InstantCommand(
                () -> _drivetrain.DRIVE_MODE_FIELD_ORIENTED = !_drivetrain.DRIVE_MODE_FIELD_ORIENTED,
                _drivetrain));
    }

    @Override
    public void robotInit() {
        _drivetrain.setDefaultCommand(new XBoxDriveCmd(_drivetrain));

        Command tagChaser = new TagChaser(_camera, _drivetrain, _drivetrain::getRobotPose);
        _autoSelector.addOption("Chase april tag command", tagChaser);

        Command tuneFrontLeftSteeringAngle = new SteeringMotorPidTuning(_drivetrain, Constants.SwerveModuleEnum.FRONT_LEFT);
        _autoSelector.addOption("Angle Controller PID Tunning", tuneFrontLeftSteeringAngle);

        Command driveForwardCommand = new DriveForwardInches(
            _drivetrain, 
            Constants.kDrivingMotorMaxSpeedMetersPerSecond,
            Units.metersToInches(2));
        _autoSelector.addOption("Forward 2m Command", driveForwardCommand);

        Command driveBackwardCommand = new DriveForwardInches(
            _drivetrain, 
            Constants.kDrivingMotorMaxSpeedMetersPerSecond,
            Units.metersToInches(2));
        _autoSelector.addOption("Backward 2m Command", driveBackwardCommand);
        
        SwerveTrajectoryTuning swerveTrajectoryTuningCmd = new SwerveTrajectoryTuning(_drivetrain);
        _autoSelector.addOption("Trajectory following Command", swerveTrajectoryTuningCmd); 

        _autoSelector.addOption("Trajectory tuning Command", swerveTrajectoryTuningCmd);
        SmartDashboard.putData("Test commands", _autoSelector);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        _autoCommand = _autoSelector.getSelected();
        if (_autoCommand != null) _autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (_autoCommand != null) _autoCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}