package frc.robot.commandGroups;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.Gyro;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.util.SmartCameraNetwork;
import frc.robot.util.SmartCameraNetwork.TargetObservation;

public class TurretLockCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final GyroSubsystem gyroSubsystem;
    private final SmartCameraNetwork cameraNetwork;
    private final SlewRateLimiter angleStepLimiter;
    
    public TurretLockCommand(TurretSubsystem turretSubsystem, GyroSubsystem gyroSubsystem, SmartCameraNetwork cameraNetwork) {
        this.turretSubsystem = turretSubsystem;
        this.gyroSubsystem = gyroSubsystem;
        this.cameraNetwork = cameraNetwork;
        this.angleStepLimiter = new SlewRateLimiter(Constants.Turret.Horizontal.MAX_TRACK_RATE_DEGREES_PER_SEC);
        addRequirements(turretSubsystem, gyroSubsystem);
    }

    @Override
    public void initialize() {
        angleStepLimiter.reset(0.0);
    }

    @Override
    public void execute() {
        Optional<HashMap<Integer,SmartCameraNetwork.TargetObservation>> optionalObservations = cameraNetwork.getAllAprilTags();

        if (optionalObservations.isEmpty()) {
            turretSubsystem.setHorizontalMotor(0.0);
            angleStepLimiter.reset(0.0);
            return;
        }

        // Find nearest april tag (usually best?) for calculating aprox center of HUB
        HashMap<Integer, TargetObservation> observations = optionalObservations.get();
        double bestDist = 999999.0;
        int bestID = -1;
        for (Integer key : observations.keySet()) {
            if (Constants.RelativeHubLocation.MY_APRIL_POS.containsKey(key)) {
                TargetObservation aprilTagObservation = observations.get(key);
                if (aprilTagObservation.getDistanceMeters() < bestDist) {
                    bestDist = aprilTagObservation.getDistanceMeters();
                    bestID = key;
                }
            }
        }
        if (bestID == -1) {
            turretSubsystem.setHorizontalMotor(0.0);
            angleStepLimiter.reset(0.0);
            return;
        }

        // Calcualte center of HUB
        TargetObservation bestObservation = observations.get(bestID);
        Translation2d hubOffset = Constants.RelativeHubLocation.MY_APRIL_POS.get(bestID);
        double xOffset = hubOffset.getX();
        double yOffset = hubOffset.getY();
        double zOffset = Constants.RelativeHubLocation.Z_POS;

        Transform3d robotToTarget = bestObservation.getRobotToTarget();
        Transform3d robotToOpening = robotToTarget.plus(new Transform3d(
            new Translation3d(xOffset, yOffset, zOffset),
            new Rotation3d()
        ));
        Transform3d turretToOpening = robotToOpening.plus(new Transform3d(
            new Translation3d(Constants.Turret.Offset.X, Constants.Turret.Offset.Y, Constants.Turret.Offset.Z ),
            new Rotation3d()
        ));

        // Calculate angle and speed at which to shoot the ball at
        double xAcc = this.gyroSubsystem.getXAcc();
        double yAcc = this.gyroSubsystem.getYAcc();
        double zAcc = this.gyroSubsystem.getZAcc();

        // [TODO]: um do the actual calculations

        /*
        double angleError = observation.get().getAngleDegrees();
        double desiredStep = 0.0;

        if (Math.abs(angleError) > Constants.Vision.ANGLE_DEADBAND) {
            desiredStep = MathUtil.clamp(
                angleError * Constants.Turret.Horizontal.ANGLE_TURN_PERCENTAGE,
                -1 * Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES,
                Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES
            );
        }

        turretSubsystem.turnHorizontalMotor(angleStepLimiter.calculate(desiredStep));
        */
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.stopHorizontal();
    }
}
