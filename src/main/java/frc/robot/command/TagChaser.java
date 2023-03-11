package frc.robot.command;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;

public class TagChaser extends CommandBase {

    private final PhotonCamera _camera;
    private final Drivetrain _drivetrain;
    private final Supplier<Pose2d> _poseProvider;

    private PhotonTrackedTarget _lastTarget;
    
    public TagChaser(
        PhotonCamera camera, 
        Drivetrain drivetrain, 
        Supplier<Pose2d> poseProvider) {

        _camera = camera;
        _drivetrain = drivetrain;
        _poseProvider = poseProvider;

        AutoConstants.xPositionController.setTolerance(0.2);
        AutoConstants.yPositionController.setTolerance(0.2);

        AutoConstants.omegaController.setTolerance(Units.degreesToRadians(3));
        AutoConstants.omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);        
    }

    @Override
    public void initialize() {
        _lastTarget = null;

        Pose2d robotPose = _poseProvider.get();

        VisionConstants.xController.reset(robotPose.getX());
        VisionConstants.yController.reset(robotPose.getY());
        VisionConstants.omegaController.reset(robotPose.getRotation().getRadians());

    }

    @Override
    public void execute() {
        Pose2d robotPose2d = _poseProvider.get();
        Pose3d robotPose3d = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0 /* z */, 
            new Rotation3d(
                0.0 /* roll */,
                0.0 /* pitch */,
                robotPose2d.getRotation().getRadians() /* yaw */
            )
        );

        var visionResult = _camera.getLatestResult();

        if (visionResult.hasTargets()) {
            Optional<PhotonTrackedTarget> possibleChaseTag = visionResult.getTargets().stream()
                .filter(t -> t.getFiducialId() == VisionConstants.kTagToChase)
                .filter(t -> !t.equals(_lastTarget) && t.getPoseAmbiguity() < VisionConstants.kDetectionAmbiguityTolerance)
                .findFirst();

            
            if (possibleChaseTag.isPresent()) {
                PhotonTrackedTarget target = possibleChaseTag.get();
                _lastTarget = target;

                Pose3d camPose = robotPose3d.transformBy(VisionConstants.kRobotToCamera);
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d targetPose = camPose.transformBy(camToTarget);

                Pose2d goalPose = targetPose.transformBy(VisionConstants.kTagToGoal).toPose2d();
                
                VisionConstants.xController.setGoal(goalPose.getX());
                VisionConstants.yController.setGoal(goalPose.getY());
                VisionConstants.omegaController.setGoal(goalPose.getRotation().getRadians());
            }

            if (_lastTarget == null) {
                _drivetrain.stopModules();
            } else {
                double xSpeed = VisionConstants.xController.calculate(robotPose3d.getX());

                if (VisionConstants.xController.atGoal()) {
                    xSpeed = 0;
                }

                double ySpeed = VisionConstants.yController.calculate(robotPose3d.getY());
                if (VisionConstants.yController.atGoal()) {
                    ySpeed = 0;
                }

                double omegaSpeed = VisionConstants.omegaController.calculate(robotPose2d.getRotation().getRadians());
                if (VisionConstants.omegaController.atGoal()) {
                    omegaSpeed = 0;
                }

                _drivetrain.drive(xSpeed, ySpeed, omegaSpeed);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.stopModules();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
