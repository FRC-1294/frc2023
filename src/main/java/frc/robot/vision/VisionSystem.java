package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSystem extends SubsystemBase {
    
    private final PhotonCamera _camera;
    private PhotonPipelineResult _visionPipeLineResult;

    public VisionSystem(String cameraName, NetworkTableInstance ntInstance) {
        _camera = new PhotonCamera(ntInstance, cameraName);
    }

    @Override
    public void periodic() {
        _visionPipeLineResult = _camera.getLatestResult();
    }

    public void setPipelineIndex(int pipeLineIndex) {
        _camera.setPipelineIndex(pipeLineIndex);
    }
    
    public PhotonTrackedTarget getAprilTagById(int fiducialId) {
        if (_visionPipeLineResult.hasTargets()) {
            for (PhotonTrackedTarget target : _visionPipeLineResult.targets) {
                if (fiducialId == target.getFiducialId() && 
                    target.getPoseAmbiguity() < VisionConstants.kDetectionAmbiguityTolerance) {                        
                    return target;
                }
            }
        }

        return null;
    }
}


class TagInfo {
    public double yaw;
    public double pitch;
    public Transform3d camToTarget;
    
    public TagInfo(PhotonTrackedTarget target) {
        yaw = target.getYaw();
        pitch = target.getPitch();
        camToTarget = target.getBestCameraToTarget();
    }
}
