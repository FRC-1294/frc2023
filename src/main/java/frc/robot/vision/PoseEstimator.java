package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.Constants.VisionConstants;

public class PoseEstimator extends SubsystemBase {
    
    private final PhotonCamera _camera;
    private double _previusPipelineTimestamp;

    private SwerveDrivePoseEstimator _poseEstimator;
    private final Drivetrain _drivetrain;
    
    public PoseEstimator(Drivetrain drivetrain, PhotonCamera camera) {
        _camera = camera;
        _drivetrain = drivetrain;
        
        _poseEstimator = new SwerveDrivePoseEstimator(
            Constants.kDriveKinematics, 
            _drivetrain.getGyroAngleRotation2d(),
            _drivetrain.getModulePositions(), 
            _drivetrain.getRobotPose(),
            VisionConstants.kStateStdDevs,
            VisionConstants.kVisionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // Update pose estimator with the best visible target 
        PhotonPipelineResult pipelineResult = _camera.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();

        if (resultTimestamp != _previusPipelineTimestamp && pipelineResult.hasTargets()) {
        
            _previusPipelineTimestamp = resultTimestamp;
            PhotonTrackedTarget target =  pipelineResult.getBestTarget();
            int fiducialId = target.getFiducialId();

            if (target.getPoseAmbiguity() < VisionConstants.kDetectionAmbiguityTolerance &&
                VisionConstants.kTargetPoseMap.containsKey(fiducialId) ) {
                
                // location of target in the field, i.e. targetPose in field space
                Pose3d targetPose = VisionConstants.kTargetPoseMap.get(fiducialId);
                SmartDashboard.putString("PoseEstimator_targetPose_Tag" + fiducialId, Constants.getPoseString(targetPose.toPose2d()));

                // location of the camera in relation to the april tag
                Transform3d camToTarget = target.getBestCameraToTarget();

                // location of the april tag in relation to the camera
                Transform3d targetToCam = camToTarget.inverse();

                // location of camera in the field, i.e. camPose in field space
                Pose3d camPose = targetPose.transformBy(targetToCam);

                // location of robot in the field, i.e. robotPose in field space
                Pose3d visionMeasurement = camPose.transformBy(VisionConstants.kCameraToRobot);
                SmartDashboard.putString("PoseEstimator_VisionMeasurement", Constants.getPoseString(visionMeasurement.toPose2d()));

                if (Math.sqrt(Math.pow(visionMeasurement.getX(), 2) + Math.pow(visionMeasurement.getY(), 2))
                    < VisionConstants.kMaxMeasurementMeters) {

                    _poseEstimator.addVisionMeasurement(
                        visionMeasurement.toPose2d(), 
                        resultTimestamp);    
                }
            } else {
                System.err.println("Target " + fiducialId + ", has pose ambiguity = " + target.getPoseAmbiguity() + " not processed.");
            }
        }

        // Update pose estimator with drive train sensors
        _poseEstimator.update(
            _drivetrain.getGyroAngleRotation2d(), 
            _drivetrain.getModulePositions());
    }

    public Pose2d getPose() {
        return _poseEstimator.getEstimatedPosition();
    }
}