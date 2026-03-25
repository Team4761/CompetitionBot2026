package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private static final double SINGLE_TAG_AMBIGUITY_CUTOFF = 0.20;
    private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
    private static final double FIELD_BORDER_MARGIN_METERS = 0.50;

    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 6.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);

    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout fieldLayout;
    private final List<CameraContext> cameras;
    private final Field2d visionField = new Field2d();
    private final Field2d drivetrainField = new Field2d();

    private Optional<VisionMeasurement> latestMeasurement = Optional.empty();
    private Pose2d currentPose = new Pose2d();
    private boolean enabled = true;
    private int acceptedMeasurementCount = 0;
    private int rejectedMeasurementCount = 0;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, defaultCameraConfigs());
    }

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, List<CameraConfig> cameraConfigs) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain");
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        this.cameras = new ArrayList<>();

        for (CameraConfig cameraConfig : cameraConfigs) {
            cameras.add(new CameraContext(cameraConfig, fieldLayout));
        }

        SmartDashboard.putData("Vision Field", visionField);
        SmartDashboard.putData("Vision Drivetrain Pose", drivetrainField);
    }

    private static List<CameraConfig> defaultCameraConfigs() {
        return List.of(
            new CameraConfig(Constants.Vision.LEFT_CAMERA_NAME, Constants.Vision.LEFT_CAM_TRANSFORM),
            new CameraConfig(Constants.Vision.RIGHT_CAMERA_NAME, Constants.Vision.RIGHT_CAM_TRANSFORM)
        );
    }

    @Override
    public void periodic() {
        currentPose = drivetrain.getState().Pose;
        drivetrainField.setRobotPose(currentPose);

        if (enabled) {
            Pose3d referencePose = new Pose3d(currentPose);
            for (CameraContext cameraContext : cameras) {
                for (PhotonPipelineResult result : cameraContext.camera.getAllUnreadResults()) {
                    if (!result.hasTargets()) {
                        rejectedMeasurementCount++;
                        continue;
                    }

                    if (result.getTargets().size() == 1) {
                        double ambiguity = result.getBestTarget().getPoseAmbiguity();
                        if (ambiguity < 0.0 || ambiguity >= SINGLE_TAG_AMBIGUITY_CUTOFF) {
                            rejectedMeasurementCount++;
                            continue;
                        }
                    }

                    Optional<EstimatedRobotPose> estimate = cameraContext.poseEstimator.estimateCoprocMultiTagPose(result);
                    if (estimate.isEmpty()) {
                        estimate = cameraContext.poseEstimator.estimateClosestToReferencePose(result, referencePose);
                    }
                    if (estimate.isEmpty()) {
                        rejectedMeasurementCount++;
                        continue;
                    }

                    EstimatedRobotPose estimatedRobotPose = estimate.get();
                    Pose2d estimatedPose = estimatedRobotPose.estimatedPose.toPose2d();
                    if (!isPoseOnField(estimatedPose)) {
                        rejectedMeasurementCount++;
                        continue;
                    }

                    Matrix<N3, N1> stdDevs = getEstimationStdDevs(estimatedRobotPose);
                    boolean hasInvalidStdDevs = false;
                    for (int row = 0; row < stdDevs.getNumRows(); row++) {
                        if (!Double.isFinite(stdDevs.get(row, 0))) {
                            hasInvalidStdDevs = true;
                            break;
                        }
                    }
                    if (hasInvalidStdDevs) {
                        rejectedMeasurementCount++;
                        continue;
                    }

                    VisionMeasurement measurement = new VisionMeasurement(
                        cameraContext.config.name(),
                        estimatedPose,
                        estimatedRobotPose.timestampSeconds,
                        stdDevs,
                        estimatedRobotPose.targetsUsed.size()
                    );

                    addVisionPose2d(measurement.pose(), measurement.timestampSeconds(), measurement.stdDevs());
                    latestMeasurement = Optional.of(measurement);
                    acceptedMeasurementCount++;
                }
            }
        }

        latestMeasurement.ifPresent(measurement -> visionField.setRobotPose(measurement.pose()));
        publishDashboard();
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public Pose2d getEstimatedPose() {
        return currentPose;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public Optional<VisionMeasurement> getLatestMeasurement() {
        return latestMeasurement;
    }

    public Field2d getVisionField() {
        return visionField;
    }

    public List<String> getCameraNames() {
        List<String> cameraNames = new ArrayList<>();
        for (CameraContext cameraContext : cameras) {
            cameraNames.add(cameraContext.config.name());
        }
        return cameraNames;
    }

    public void addVisionPose2d(Pose2d pose2d, double timestampSeconds) {
        addVisionPose2d(pose2d, timestampSeconds, SINGLE_TAG_STD_DEVS);
    }

    public void addVisionPose2d(Pose2d pose2d, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
        drivetrain.addVisionMeasurement(pose2d, timestampSeconds, visionStdDevs);
    }

    public void CameraTeamColorSwitcher(boolean isBlue) {
        int pipelineIndex = isBlue ? 0 : 1;
        for (CameraContext cameraContext : cameras) {
            cameraContext.camera.setPipelineIndex(pipelineIndex);
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimate) {
        Matrix<N3, N1> stdDevs = SINGLE_TAG_STD_DEVS;
        int seenTags = 0;
        double totalDistanceMeters = 0.0;

        for (PhotonTrackedTarget target : estimate.targetsUsed) {
            Optional<edu.wpi.first.math.geometry.Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }

            seenTags++;
            totalDistanceMeters += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimate.estimatedPose.toPose2d().getTranslation());
        }

        if (seenTags == 0) {
            return SINGLE_TAG_STD_DEVS;
        }

        double averageDistanceMeters = totalDistanceMeters / seenTags;
        if (seenTags > 1) {
            stdDevs = MULTI_TAG_STD_DEVS;
        }

        if (seenTags == 1 && averageDistanceMeters > MAX_SINGLE_TAG_DISTANCE_METERS) {
            return VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        return stdDevs.times(1.0 + (averageDistanceMeters * averageDistanceMeters / 30.0));
    }

    private boolean isPoseOnField(Pose2d pose) {
        return pose.getX() >= -FIELD_BORDER_MARGIN_METERS
            && pose.getX() <= fieldLayout.getFieldLength() + FIELD_BORDER_MARGIN_METERS
            && pose.getY() >= -FIELD_BORDER_MARGIN_METERS
            && pose.getY() <= fieldLayout.getFieldWidth() + FIELD_BORDER_MARGIN_METERS;
    }

    private void publishDashboard() {
        SmartDashboard.putBoolean("Vision Enabled", enabled);
        SmartDashboard.putNumber("Vision Camera Count", cameras.size());
        SmartDashboard.putNumber("Vision Accepted Measurements", acceptedMeasurementCount);
        SmartDashboard.putNumber("Vision Rejected Measurements", rejectedMeasurementCount);
        SmartDashboard.putBoolean("Vision Has Measurement", latestMeasurement.isPresent());
        SmartDashboard.putNumber("Vision Estimated Pose X", currentPose.getX());
        SmartDashboard.putNumber("Vision Estimated Pose Y", currentPose.getY());
        SmartDashboard.putNumber("Vision Estimated Pose Degrees", currentPose.getRotation().getDegrees());

        if (latestMeasurement.isPresent()) {
            VisionMeasurement measurement = latestMeasurement.get();
            SmartDashboard.putString("Vision Last Camera", measurement.cameraName());
            SmartDashboard.putNumber("Vision Last Timestamp", measurement.timestampSeconds());
            SmartDashboard.putNumber("Vision Last Tag Count", measurement.tagCount());
            SmartDashboard.putNumber("Vision Last Pose X", measurement.pose().getX());
            SmartDashboard.putNumber("Vision Last Pose Y", measurement.pose().getY());
            SmartDashboard.putNumber("Vision Last Pose Degrees", measurement.pose().getRotation().getDegrees());
        } else {
            SmartDashboard.putString("Vision Last Camera", "none");
            SmartDashboard.putNumber("Vision Last Timestamp", 0.0);
            SmartDashboard.putNumber("Vision Last Tag Count", 0);
            SmartDashboard.putNumber("Vision Last Pose X", 0.0);
            SmartDashboard.putNumber("Vision Last Pose Y", 0.0);
            SmartDashboard.putNumber("Vision Last Pose Degrees", 0.0);
        }
    }

    public static record CameraConfig(String name, Transform3d robotToCamera) {
        public CameraConfig {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(robotToCamera, "robotToCamera");
        }
    }

    public static record VisionMeasurement(
        String cameraName,
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs,
        int tagCount
    ) {}

    private static final class CameraContext {
        private final CameraConfig config;
        private final PhotonCamera camera;
        private final PhotonPoseEstimator poseEstimator;

        private CameraContext(CameraConfig config, AprilTagFieldLayout fieldLayout) {
            this.config = config;
            this.camera = new PhotonCamera(config.name());
            this.poseEstimator = new PhotonPoseEstimator(fieldLayout, config.robotToCamera());
        }
    }
}
