package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private static final double GRAVITY_METERS_PER_SEC_SQ = 9.80665;
    private static final double LAUNCH_LEAD_TIME_SECONDS = 0.08;
    private static final double MAX_YAW_ACCEL_RAD_PER_SEC_SQ = 20.0;
    private static final int HEIGHT_COUPLING_ITERATIONS = 5;

    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonCamera camera = new PhotonCamera(Constants.Vision.DEFAULT_CAMERA_NAME);
    private final SlewRateLimiter angleStepLimiter =
        new SlewRateLimiter(Constants.Turret.Horizontal.MAX_TRACK_RATE_DEGREES_PER_SEC);
    private Optional<HashMap<Integer, AprilTagObservation>> currentAprilTagObservations = Optional.empty();
    private Optional<TurretAimSuggestion> latestTurretAimSuggestion = Optional.empty();
    private double angleToAprilCode = 0;
    private double distToAprilCode = 0;
    private boolean tracking = true;
    private boolean readyToFire = false;
    private double previousOmegaRadPerSec = 0.0;
    private double previousTimestampSec = 0.0;
    private boolean hasPreviousOmega = false;
    private double latestYawAccelRadPerSecSq = 0.0;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void disenableTracker() {
        this.tracking = !this.tracking;
        System.out.println(this.tracking);
    }

    public boolean isTracking() {
        return tracking;
    }

    @Override
    public void periodic() {
        processAprilTags();
        updateYawAcceleration();
        latestTurretAimSuggestion = calculateTurretAimSuggestion();
        publishDashboardAim();
    }

    public double getAngleToAprilCode() {
        return angleToAprilCode;
    }

    public double getDistToAprilCode() {
        return distToAprilCode;
    }

    public void processAprilTags() {
        Optional<AprilTagObservation> observation = getBestHubObservation();
        if (observation.isPresent()) {
            this.angleToAprilCode = observation.get().angleDegrees();
            this.distToAprilCode = observation.get().distanceMeters();
        } else {
            this.angleToAprilCode = 0;
            this.distToAprilCode = 0;
        }
    }

    public boolean shootSetUpComplete(boolean isReadyToFire) {
        readyToFire = isReadyToFire;
        return isReadyToFire;
    }

    public void resetTurretTrackingState() {
        angleStepLimiter.reset(0.0);
        previousOmegaRadPerSec = 0.0;
        previousTimestampSec = 0.0;
        hasPreviousOmega = false;
        latestYawAccelRadPerSecSq = 0.0;
    }

    public Optional<TurretAimOutput> getTurretAimOutput() {
        if (latestTurretAimSuggestion.isEmpty()) {
            angleStepLimiter.reset(0.0);
            return Optional.empty();
        }

        TurretAimSuggestion aimSuggestion = latestTurretAimSuggestion.get();
        double desiredStep = 0.0;
        if (Math.abs(aimSuggestion.horizontalErrorDegrees()) > Constants.Vision.ANGLE_DEADBAND) {
            desiredStep = MathUtil.clamp(
                aimSuggestion.horizontalErrorDegrees() * Constants.Turret.Horizontal.ANGLE_TURN_PERCENTAGE,
                -Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES,
                Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES
            );
        }

        return Optional.of(new TurretAimOutput(
            angleStepLimiter.calculate(desiredStep),
            aimSuggestion.verticalLaunchAngleDegrees()
        ));
    }

    public Color seesAprilTag() {
        Color color = null;
        if (this.angleToAprilCode == 0 && this.distToAprilCode == 0) {
            color = new Color(255, 50, 50);
        } else {
            color = new Color(255, 255, 50);
            if (readyToFire) {
                color = new Color(50, 255, 50);
            }
            if (!tracking)
            {
                color = new Color(127,50,255);
            }
        }
        return color;
    }

    public static record TurretAimOutput(double horizontalTurnDegrees, double verticalLaunchAngleDegrees) {}
    public static record TurretAimSuggestion(double horizontalErrorDegrees, double verticalLaunchAngleDegrees) {}

    private Optional<AprilTagObservation> getBestHubObservation() {
        Optional<HashMap<Integer, AprilTagObservation>> optionalObservations = getAllAprilTags();
        if (optionalObservations.isEmpty()) {
            return Optional.empty();
        }

        AprilTagObservation bestObservation = null;
        for (AprilTagObservation observation : optionalObservations.get().values()) {
            if (bestObservation == null || observation.distanceMeters() < bestObservation.distanceMeters()) {
                bestObservation = observation;
            }
        }
        return Optional.ofNullable(bestObservation);
    }

    private Optional<HashMap<Integer, AprilTagObservation>> getAllAprilTags() {
        var hubAprilTagPositions = Constants.RelativeHubLocation.hubAprilTagPositions();
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        if (unreadResults.isEmpty()) {
            if (currentAprilTagObservations.isPresent()) {
                currentAprilTagObservations = filterForCurrentAlliance(
                    currentAprilTagObservations.get(),
                    hubAprilTagPositions
                );
            }
            return currentAprilTagObservations;
        }

        PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
        HashMap<Integer, AprilTagObservation> observations = new HashMap<>();
        if (latestResult.hasTargets()) {
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                if (!hubAprilTagPositions.containsKey(target.getFiducialId())) {
                    continue;
                }

                Transform3d cameraToTarget = target.getBestCameraToTarget();
                if (cameraToTarget == null) {
                    continue;
                }

                AprilTagObservation observation = new AprilTagObservation(
                    target.getFiducialId(),
                    cameraToTarget,
                    -Math.toDegrees(Math.atan2(cameraToTarget.getY(), cameraToTarget.getX())),
                    Math.hypot(cameraToTarget.getX(), cameraToTarget.getY())
                );
                AprilTagObservation existingObservation = observations.get(target.getFiducialId());
                if (existingObservation == null || observation.distanceMeters() < existingObservation.distanceMeters()) {
                    observations.put(target.getFiducialId(), observation);
                }
            }
        }

        currentAprilTagObservations = observations.isEmpty() ? Optional.empty() : Optional.of(observations);
        return currentAprilTagObservations;
    }

    private static record AprilTagObservation(
        int fiducialId,
        Transform3d cameraToTarget,
        double angleDegrees,
        double distanceMeters
    ) {}

    private void updateYawAcceleration() {
        double omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
        double nowSec = Timer.getFPGATimestamp();
        double yawAccel = 0.0;
        if (hasPreviousOmega) {
            double dt = nowSec - previousTimestampSec;
            if (dt > 1e-6) {
                yawAccel = MathUtil.clamp(
                    (omega - previousOmegaRadPerSec) / dt,
                    -MAX_YAW_ACCEL_RAD_PER_SEC_SQ,
                    MAX_YAW_ACCEL_RAD_PER_SEC_SQ
                );
            }
        }

        hasPreviousOmega = true;
        previousOmegaRadPerSec = omega;
        previousTimestampSec = nowSec;
        latestYawAccelRadPerSecSq = yawAccel;
    }

    private Optional<TurretAimSuggestion> calculateTurretAimSuggestion() {
        if (!tracking) {
            return Optional.empty();
        }

        Optional<AprilTagObservation> optionalObservation = getBestHubObservation();
        if (optionalObservation.isEmpty()) {
            return Optional.empty();
        }

        AprilTagObservation bestObservation = optionalObservation.get();
        Translation2d hubOffset =
            Constants.RelativeHubLocation.hubAprilTagPositions().get(bestObservation.fiducialId());
        double tagToHubOpeningZMeters =
            Units.inchesToMeters(Constants.RelativeHubLocation.Z_POS) - FieldConstants.AprilTag.APRILTAG_HUB_HEIGHT;

        var speeds = drivetrain.getState().Speeds;
        double vxRobot = speeds.vxMetersPerSecond;
        double vyRobot = speeds.vyMetersPerSecond;
        double omegaAtLaunch = speeds.omegaRadiansPerSecond + latestYawAccelRadPerSecSq * LAUNCH_LEAD_TIME_SECONDS;
        double turretVelocityX = vxRobot - omegaAtLaunch * Constants.Turret.Offset.Y;
        double turretVelocityY = vyRobot + omegaAtLaunch * Constants.Turret.Offset.X;
        double speedMagnitude = Constants.Turret.MAX_SPEED_MEASURED_MpS;

        double horizontalErrorDegrees = 0.0;
        double launchAngleDegrees = Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES;

        for (int i = 0; i < HEIGHT_COUPLING_ITERATIONS; i++) {
            double cameraOffsetZMeters = Constants.Vision.CAMERA_OFFSET_FROM_TURRET_Z.apply(launchAngleDegrees);
            Translation3d tagToOpening = tagToOpeningTranslation(hubOffset, tagToHubOpeningZMeters);

            Transform3d cameraToOpening = bestObservation.cameraToTarget().plus(new Transform3d(
                tagToOpening,
                new Rotation3d()
            ));
            Transform3d turretToOpening = cameraToOpening.plus(new Transform3d(
                new Translation3d(
                    Constants.Vision.CAMERA_OFFSET_FROM_TURRET_X,
                    Constants.Vision.CAMERA_OFFSET_FROM_TURRET_Y,
                    cameraOffsetZMeters
                ),
                new Rotation3d()
            ));

            double x = turretToOpening.getX();
            double y = turretToOpening.getY();
            double z = turretToOpening.getZ();

            OptionalDouble flightTime = solveFlightTimeSeconds(
                x,
                y,
                z,
                turretVelocityX,
                turretVelocityY,
                speedMagnitude
            );
            if (flightTime.isEmpty()) {
                return Optional.empty();
            }

            double t = flightTime.getAsDouble();
            double shotVx = (x - turretVelocityX * t) / t;
            double shotVy = (y - turretVelocityY * t) / t;
            double shotVz = (z + 0.5 * GRAVITY_METERS_PER_SEC_SQ * t * t) / t;

            horizontalErrorDegrees = -Math.toDegrees(Math.atan2(shotVy, shotVx));
            launchAngleDegrees = MathUtil.clamp(
                Math.toDegrees(Math.atan2(shotVz, Math.hypot(shotVx, shotVy))),
                Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES,
                Constants.Turret.Vertical.MAX_LAUNCH_ANGLE_DEGREES
            );
        }

        return Optional.of(new TurretAimSuggestion(horizontalErrorDegrees, launchAngleDegrees));
    }

    private static Optional<HashMap<Integer, AprilTagObservation>> filterForCurrentAlliance(
        HashMap<Integer, AprilTagObservation> observations,
        java.util.Map<Integer, Translation2d> hubAprilTagPositions
    ) {
        HashMap<Integer, AprilTagObservation> filteredObservations = new HashMap<>(observations);
        filteredObservations.keySet().removeIf(tagId -> !hubAprilTagPositions.containsKey(tagId));
        return filteredObservations.isEmpty() ? Optional.empty() : Optional.of(filteredObservations);
    }

    private void publishDashboardAim() {
        if (latestTurretAimSuggestion.isPresent()) {
            TurretAimSuggestion aimSuggestion = latestTurretAimSuggestion.get();
            SmartDashboard.putBoolean("Turret Vision Has Target", true);
            SmartDashboard.putNumber("Turret Vision Horizontal Turn Degrees", aimSuggestion.horizontalErrorDegrees());
            SmartDashboard.putNumber("Turret Vision Vertical Launch Degrees", aimSuggestion.verticalLaunchAngleDegrees() - Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES);
            return;
        }

        SmartDashboard.putBoolean("Turret Vision Has Target", false);
        SmartDashboard.putNumber("Turret Vision Horizontal Turn Degrees", 0.0);
        SmartDashboard.putNumber("Turret Vision Vertical Launch Degrees", 0.0);
    }

    private static Translation3d tagToOpeningTranslation(Translation2d hubOffsetInches, double openingZMeters) {
        double offsetXMeters = Units.inchesToMeters(hubOffsetInches.getX());
        double offsetYMeters = Units.inchesToMeters(hubOffsetInches.getY());

        Translation2d faceNormal = faceNormalForTag(hubOffsetInches);
        Translation2d faceLeft = new Translation2d(-faceNormal.getY(), faceNormal.getX());
        Translation2d tagToCenter = new Translation2d(-offsetXMeters, -offsetYMeters);

        return new Translation3d(
            dot(tagToCenter, faceNormal),
            dot(tagToCenter, faceLeft),
            openingZMeters
        );
    }

    private static Translation2d faceNormalForTag(Translation2d hubOffsetInches) {
        double x = hubOffsetInches.getX();
        double y = hubOffsetInches.getY();

        if (Math.abs(y) >= Math.abs(x)) {
            return new Translation2d(0.0, Math.signum(y));
        }
        return new Translation2d(Math.signum(x), 0.0);
    }

    private static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    private static OptionalDouble solveFlightTimeSeconds(
        double x,
        double y,
        double z,
        double vx,
        double vy,
        double speedMagnitude
    ) {
        final double minTime = 0.02;
        final double maxTime = 3.0;
        final double dt = 0.02;

        double prevT = minTime;
        double prevF = speedErrorForTime(x, y, z, vx, vy, speedMagnitude, prevT);
        if (Math.abs(prevF) < 1e-6) {
            return OptionalDouble.of(prevT);
        }

        for (double t = minTime + dt; t <= maxTime; t += dt) {
            double f = speedErrorForTime(x, y, z, vx, vy, speedMagnitude, t);
            if (Math.abs(f) < 1e-6) {
                return OptionalDouble.of(t);
            }

            if ((prevF < 0.0 && f > 0.0) || (prevF > 0.0 && f < 0.0)) {
                double left = prevT;
                double right = t;
                for (int i = 0; i < 40; i++) {
                    double mid = 0.5 * (left + right);
                    double midF = speedErrorForTime(x, y, z, vx, vy, speedMagnitude, mid);
                    if (Math.abs(midF) < 1e-6) {
                        return OptionalDouble.of(mid);
                    }
                    if ((prevF < 0.0 && midF > 0.0) || (prevF > 0.0 && midF < 0.0)) {
                        right = mid;
                    } else {
                        left = mid;
                        prevF = midF;
                    }
                }
                return OptionalDouble.of(0.5 * (left + right));
            }

            prevT = t;
            prevF = f;
        }

        return OptionalDouble.empty();
    }

    private static double speedErrorForTime(
        double x,
        double y,
        double z,
        double vx,
        double vy,
        double speedMagnitude,
        double t
    ) {
        double neededVx = (x - vx * t) / t;
        double neededVy = (y - vy * t) / t;
        double neededVz = (z + 0.5 * GRAVITY_METERS_PER_SEC_SQ * t * t) / t;
        double neededSpeedSq = neededVx * neededVx + neededVy * neededVy + neededVz * neededVz;
        return neededSpeedSq - speedMagnitude * speedMagnitude;
    }
}
