package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SmartCameraNetwork {
    private static final String DEFAULT_CONFIG = "vision_cameras.json";
    private final List<CameraEntry> cameras;
    private Optional<HashMap<Integer, TargetObservation>> currentAprilTagsDetected;

    public SmartCameraNetwork(Builder builder) {
        File configFile = builder.configFile != null
            ? builder.configFile
            : new File(Filesystem.getDeployDirectory(), DEFAULT_CONFIG);
        this.cameras = loadCameras(configFile);
    }

    public Optional<HashMap<Integer, TargetObservation>> getAllAPrilTags() {
        HashMap<Integer, TargetObservation> observations = new HashMap<>();

        for (CameraEntry entry : cameras) {
            PhotonPipelineResult results = entry.camera.getLatestResult();
            if (!results.hasTargets()) {
                continue;
            }

            for (PhotonTrackedTarget target : results.getTargets()) {
                Transform3d cameraToTarget = target.getBestCameraToTarget();
                if (cameraToTarget == null) {
                    continue;
                }

                Transform3d robotToTarget = getRelativePos(entry.robotToCamera, cameraToTarget);
                double distance = Math.hypot(robotToTarget.getX(), robotToTarget.getY());
                if (observations.containsKey(target.getFiducialId())) {
                    TargetObservation existing = observations.get(target.getFiducialId());
                    double existingDistance = existing.getDistanceMeters();
                    if (distance >= existingDistance) {
                        continue;
                    }
                }
                observations.put(target.getFiducialId(), new TargetObservation(entry.name, robotToTarget, cameraToTarget));
            }
        }

        this.currentAprilTagsDetected = observations.isEmpty() ? Optional.empty() : Optional.of(observations);
        return this.currentAprilTagsDetected;
    }

    public Optional<TargetObservation> getBestObservation(int fiducialId) {
        if(currentAprilTagsDetected.isEmpty()) { return Optional.empty(); }
        
        return Optional.ofNullable(
            this.currentAprilTagsDetected.get().get(fiducialId)
        );
    }

    private static Transform3d getRelativePos(Transform3d robotToCamera, Transform3d cameraToTarget) {
        Pose3d origin = new Pose3d();
        Pose3d cameraPose = origin.transformBy(robotToCamera);
        Pose3d targetPose = cameraPose.transformBy(cameraToTarget);
        return new Transform3d(origin, targetPose);
    }

    private static List<CameraEntry> loadCameras(File configFile) {
        ObjectMapper mapper = new ObjectMapper();
        List<CameraEntry> entries = new ArrayList<>();

        try {
            VisionConfig config = mapper.readValue(configFile, VisionConfig.class);
            if (config == null || config.cameras == null || config.cameras.isEmpty()) {
                throw new IOException("No cameras defined in " + configFile.getAbsolutePath());
            }

            for (CameraConfig camera : config.cameras) {
                Transform3d robotToCamera = new Transform3d(
                    new Translation3d(camera.position.x, camera.position.y, camera.position.z),
                    new Rotation3d(
                        Math.toRadians(camera.rotation_rpy.roll),
                        Math.toRadians(camera.rotation_rpy.pitch),
                        Math.toRadians(camera.rotation_rpy.yaw)
                    )
                );
                entries.add(new CameraEntry(camera.name, new PhotonCamera(camera.name), robotToCamera));
            }
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load vision camera config, using default camera: " + ex.getMessage(), ex.getStackTrace());
            entries.add(defaultCamera());
        }

        return entries;
    }

    private static CameraEntry defaultCamera() {
        Transform3d robotToCamera = new Transform3d();
        return new CameraEntry(Constants.Vision.DEFAULT_CAM, new PhotonCamera(Constants.Vision.DEFAULT_CAM), robotToCamera);
    }

    private static class CameraEntry {
        private final String name;
        private final PhotonCamera camera;
        private final Transform3d robotToCamera;

        private CameraEntry(String name, PhotonCamera camera, Transform3d robotToCamera) {
            this.name = name;
            this.camera = camera;
            this.robotToCamera = robotToCamera;
        }
    }

    public static class TargetObservation {
        private final String cameraName;
        private final Transform3d robotToTarget;
        private final Transform3d cameraToTarget;
        private final double angleDegrees;
        private final double distanceMeters;

        private TargetObservation(String cameraName, Transform3d robotToTarget, Transform3d cameraToTarget) {
            this.cameraName = cameraName;
            this.robotToTarget = robotToTarget;
            this.cameraToTarget = cameraToTarget;
            this.angleDegrees = -Math.toDegrees(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
            this.distanceMeters = -Math.hypot(robotToTarget.getX(), robotToTarget.getY());
        }

        public String getCameraName() { return cameraName; }
        public Transform3d getRobotToTarget() { return robotToTarget; }
        public Transform3d getCameraToTarget() { return cameraToTarget; }
        public double getAngleDegrees() { return angleDegrees; }
        public double getDistanceMeters() { return distanceMeters; }
    }

    private static class VisionConfig {
        public List<CameraConfig> cameras;
    }

    private static class CameraConfig {
        public String name;
        public Position position;
        public RotationRpy rotation_rpy;
    }

    private static class Position {
        public double x;
        public double y;
        public double z;
    }

    private static class RotationRpy {
        public double roll;
        public double pitch;
        public double yaw;
    }

    public static class Builder {
        private File configFile;

        public static Builder newInstance() { return new Builder(); }

        public Builder() {}

        public Builder configFile(File configFile) { this.configFile = configFile; return this; }

        public SmartCameraNetwork build() { return new SmartCameraNetwork(this); }
    }
}
