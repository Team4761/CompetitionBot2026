package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartCameraNetwork;
import frc.robot.util.SmartCameraNetwork.TargetObservation;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final SmartCameraNetwork smartCamera = SmartCameraNetwork.Builder.newInstance().build();
    private double angleToAprilCode = 0;
    private double distToAprilCode = 0;
    private boolean tracking = false;

    public void disenableTracker() {
        this.tracking = !this.tracking;
        System.out.println(this.tracking);
    }

    public boolean isTracking() {
        return tracking;
    }

    public double getAngleToAprilCode() {
        return angleToAprilCode;
    }

    public double getDistToAprilCode() {
        return distToAprilCode;
    }

    public double getAdjustedDistToAprilCode() {
        return distToAprilCode - Constants.Vision.BACKUP_DIST;
    }

    public void processAprilTags() {
        Optional<TargetObservation> observation = smartCamera.getBestObservation(22);
        if (observation.isPresent()) {
            this.angleToAprilCode = observation.get().getAngleDegrees();
            this.distToAprilCode = observation.get().getDistanceMeters();
        } else {
            this.angleToAprilCode = 0;
            this.distToAprilCode = 0;
        }
    }
}
