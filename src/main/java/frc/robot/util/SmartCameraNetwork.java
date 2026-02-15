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
    private static final String DEFAULT_CONFIG = "vision_cameras.json"; // this file is a config file in the deploy directory that specifies the cameras and their positions on the robot.
    private final List<CameraEntry> cameras; // list of all cameras
    private Optional<HashMap<Integer, TargetObservation>> currentAprilTagsDetected; // a list of all AprilTags currently detected by the cameras, with IDs and positions tied to them

    /**
     * Constructor. Seems to load all the cameras via loadCameras and configures them into the network.
     */
    public SmartCameraNetwork(Builder builder) {
        File configFile = builder.configFile != null
            ? builder.configFile
            : new File(Filesystem.getDeployDirectory(), DEFAULT_CONFIG);
        this.cameras = loadCameras(configFile);
    }

    /**
     * This function gets all the AprilTags, including relative Position to the robot & camera, along with their IDs.
     * @return A map of all the AprilTags currently detected by the cameras, with IDs and positions tied to them. If no AprilTags are detected, returns an empty Optional.
     */
    public Optional<HashMap<Integer, TargetObservation>> getAllAprilTags() {
        // Creates a HashMap for all the observations, where the key is AprilTag IDs and the value is a TargetObservation object.
        HashMap<Integer, TargetObservation> observations = new HashMap<>();

        // Loops through all cameras in the network, and gets the latest result from each of them. If the camera has no targets, it continues to the next camera. 
        for (CameraEntry entry : cameras) {
            PhotonPipelineResult results = entry.camera.getLatestResult();
            if (!results.hasTargets()) {
                continue;
            }

            // For each target detected by a camera, it gets the best one.
            for (PhotonTrackedTarget target : results.getTargets()) {
                Transform3d cameraToTarget = target.getBestCameraToTarget();

                if (cameraToTarget == null) {
                    continue;
                }
                // Calculates the relative position of the target with respect to the robot by using the robotToCamera and cameraToTarget transforms.
                Transform3d robotToTarget = getRelativePos(entry.robotToCamera, cameraToTarget);
                double distance = Math.hypot(robotToTarget.getX(), robotToTarget.getY());

                if (observations.containsKey(target.getFiducialId())) {
                    
                    TargetObservation existing = observations.get(target.getFiducialId());
                    double existingDistance = existing.getDistanceMeters();
                    if (distance >= existingDistance) { // If the new observation is farther than the existing one, we ignore it and keep the closer one.
                        continue;
                    }
                }

                // If the target is not already in the observations map, or if the new observation is closer than the existing one, we add/update it in the map.
                observations.put(target.getFiducialId(), new TargetObservation(entry.name, robotToTarget, cameraToTarget));
            }
        }
        // If there are no observations, we set currentAprilTagsDetected to an empty Optional. Otherwise, we set it to an Optional containing the observations map.
        this.currentAprilTagsDetected = observations.isEmpty() ? Optional.empty() : Optional.of(observations);
        
        return this.currentAprilTagsDetected;
    }
    /**
     * Gets the best observation for a given AprilTag ID. If the tag is not currently detected, returns an empty Optional.
     * @param fiducialId The ID of the AprilTag to get the observation for.
     * @return The best observation for the given AprilTag ID, or an empty Optional if not detected.
     */
    public Optional<TargetObservation> getBestObservation(int fiducialId) {
        if(currentAprilTagsDetected.isEmpty()) { return Optional.empty(); }
        
        return Optional.ofNullable(
            this.currentAprilTagsDetected.get().get(fiducialId)
        );
    }
    /**
     * Gets the relative position based on the robotToCamera and cameraToTarget transforms. This is done by transforming the origin by the robotToCamera to get the camera's position, then transforming that by the cameraToTarget to get the target's position relative to the robot.
     * @param robotToCamera The transform from the robot to the camera.
     * @param cameraToTarget The transform from the camera to the target.
     * @return The relative position of the target with respect to the robot.
     */
    private static Transform3d getRelativePos(Transform3d robotToCamera, Transform3d cameraToTarget) {
        Pose3d origin = new Pose3d();
        Pose3d cameraPose = origin.transformBy(robotToCamera);
        Pose3d targetPose = cameraPose.transformBy(cameraToTarget);
        return new Transform3d(origin, targetPose);
    }
    /**
     * Loads the cameras from the given config file. The config file should be a JSON file with the following structure:
     * @param configFile The file to load the cameras from.
     * @return A list of CameraEntry objects representing the loaded cameras with translations.
     */
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

    /**
     * This functions returns a default CameraEntry with the same name from the Constants file and default Transform3d. This is used as a fallback if loading the cameras from the config file fails.
     * @return A default CameraEntry with the same name from the Constants file and default Transform3d. 
     */
    private static CameraEntry defaultCamera() {
        Transform3d robotToCamera = new Transform3d();
        return new CameraEntry(Constants.Vision.DEFAULT_CAM, new PhotonCamera(Constants.Vision.DEFAULT_CAM), robotToCamera);
    }

    // This is a private static class that represents an entry for a camera in the SmartCameraNetwork. It contains the name of the camera, the PhotonCamera object, and the Transform3d representing the position of the camera relative to the robot.
    private static class CameraEntry {
        private final String name;
        private final PhotonCamera camera;
        private final Transform3d robotToCamera;
        /**
         * Constructor for CameraEntry. Initializes the name, camera, and robotToCamera fields.
         * @param name The name of the camera, used for identification and matching with the config file.
         * @param camera The PhotonCamera object representing the camera itself, used for getting vision data.
         * @param robotToCamera The Transform3d representing the position and orientation of the camera relative to the robot, used for calculating target positions.
         */
        private CameraEntry(String name, PhotonCamera camera, Transform3d robotToCamera) {
            this.name = name;
            this.camera = camera;
            this.robotToCamera = robotToCamera;
        }
    }

    //This is a public static class that represents an observation of a target (AprilTag) by the SmartCameraNetwork. It contains the name of the camera that observed the target, the Transform3d representing the position of the target relative to the robot, the Transform3d representing the position of the target relative to the camera, the angle to the target in degrees, and the distance to the target in meters.
    public static class TargetObservation {
        private final String cameraName;
        private final Transform3d robotToTarget;
        private final Transform3d cameraToTarget;
        private final double angleDegrees;
        private final double distanceMeters;
        /**
         * Constructor for TargetObservation. Initializes the cameraName, robotToTarget, cameraToTarget, angleDegrees, and distanceMeters fields.
         * @param cameraName The name of the camera that observed the target.
         * @param robotToTarget The Transform3d representing the position of the target relative to the robot.
         * @param cameraToTarget The Transform3d representing the position of the target relative to the camera.
         */
        private TargetObservation(String cameraName, Transform3d robotToTarget, Transform3d cameraToTarget) {
            this.cameraName = cameraName;
            this.robotToTarget = robotToTarget;
            this.cameraToTarget = cameraToTarget;
            this.angleDegrees = -Math.toDegrees(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
            this.distanceMeters = -Math.hypot(robotToTarget.getX(), robotToTarget.getY());
        }
        // Get functions for all the fields
        public String getCameraName() { return cameraName; }
        public Transform3d getRobotToTarget() { return robotToTarget; }
        public Transform3d getCameraToTarget() { return cameraToTarget; }
        public double getAngleDegrees() { return angleDegrees; }
        public double getDistanceMeters() { return distanceMeters; }
    }
    // These are private static classes used for parsing the JSON config file. They represent the structure of the JSON file, which should have a list of cameras, each with a name, position, and rotation. (This one I don't know if it's accurate)
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
    // This is the public static Builder class for the SmartCameraNetwork. It allows for building a SmartCameraNetwork with optional parameters, such as a custom config file.
    public static class Builder {
        private File configFile;

        public static Builder newInstance() { return new Builder(); }

        public Builder() {}

        public Builder configFile(File configFile) { this.configFile = configFile; return this; }

        public SmartCameraNetwork build() { return new SmartCameraNetwork(this); }
    }
}
