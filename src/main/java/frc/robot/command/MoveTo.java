package frc.robot.command;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.Constants.AutoConstants;

public class MoveTo extends CommandBase {        
    private final Drivetrain _drivetrain;

    private final Pose2d _startPose;
    private final Pose2d _endPose;
    private final List<Translation2d> _midPoints;

    private final TrajectoryConfig _trajectoryConfig;
    private final Trajectory _trajectory;
    private Command _trajectoryFollowingCommand;

    public MoveTo(
        Drivetrain drivetrain, 
        Transform2d theTransform) {
        this(drivetrain, drivetrain.getRobotPose().transformBy(theTransform));
    }
    
    public MoveTo(
        Drivetrain drivetrain, 
        Pose2d desiredLocation) {

        _drivetrain = drivetrain;
        addRequirements(_drivetrain);

        _startPose = _drivetrain.getRobotPose();        
        printPose("MoveTo_Constructor_startPose", _startPose);

        _endPose = desiredLocation;
        printPose("MoveTo_Constructor_endPose", _endPose);

        _midPoints = makeMidPoints();

        _trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kDrivingMaxSpeedMetersPerSecond, 
            AutoConstants.kDrivingMaxAccelerationMetersPerSecondSquared);
                        
        _trajectoryConfig.setKinematics(Constants.kDriveKinematics);
    
        _trajectory = TrajectoryGenerator.generateTrajectory(
            _startPose, 
            _midPoints, 
            _endPose, 
            _trajectoryConfig);
    }

    private List<Translation2d> makeMidPoints() {
        Translation2d mid = new Translation2d(
            (_startPose.getX() + _endPose.getX()) /2, 
            (_startPose.getY() + _endPose.getY()) /2);
        return List.of(mid);
    }

    private void printPose(String desc, Pose2d pose) {
        SmartDashboard.putString(desc, Constants.getPoseString(pose));
    }

    @Override
    public void initialize() { 
        AutoConstants.xPositionController.setTolerance(AutoConstants.kPositionToleranceMeters);
        AutoConstants.xPositionController.reset();

        AutoConstants.yPositionController.setTolerance(AutoConstants.kPositionToleranceMeters);
        AutoConstants.yPositionController.reset();

        AutoConstants.omegaController.enableContinuousInput(-Math.PI, Math.PI);
        AutoConstants.omegaController.setTolerance(AutoConstants.kAngleToleranceRadians);
        AutoConstants.omegaController.reset(_startPose.getRotation().getRadians());

        _trajectoryFollowingCommand = new SwerveControllerCommand(
            _trajectory, 
            _drivetrain::getRobotPose, 
            Constants.kDriveKinematics, 
            AutoConstants.xPositionController,
            AutoConstants.yPositionController, 
            AutoConstants.omegaController, 
            _drivetrain::setModuleStates, 
            _drivetrain);
        
        _trajectoryFollowingCommand.schedule();
    }

    @Override
    public void execute() {
        printPose("MoveTo_execute_currentPose", _drivetrain.getRobotPose());
        printPose("MoveTo_execute_endPose", _endPose);
        SmartDashboard.putNumberArray("MoveTo_execute_Error",
            new double[] {
                AutoConstants.xPositionController.getPositionError(),
                AutoConstants.yPositionController.getPositionError(),
                AutoConstants.omegaController.getPositionError()
            });
        AutoConstants.debugPositionControllers("MoveTo_execute_controller_gains");
    }

    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
        SmartDashboard.putBooleanArray("MoveTo_isFinished_atSetpoint",
            new boolean[] {
                AutoConstants.xPositionController.atSetpoint(),
                AutoConstants.yPositionController.atSetpoint(),
                AutoConstants.omegaController.atSetpoint()
            });

        return  AutoConstants.xPositionController.atSetpoint() && 
                    AutoConstants.yPositionController.atSetpoint() && 
                    AutoConstants.omegaController.atSetpoint();
    }
}