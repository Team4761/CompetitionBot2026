package frc.robot.commandGroups;

import java.util.HashMap;
import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.util.SmartCameraNetwork;
import frc.robot.util.SmartCameraNetwork.TargetObservation;

public class TurretLockCommand extends Command {
    private static final double GRAVITY_METERS_PER_SEC_SQ = 9.80665;
    private static final double LAUNCH_LEAD_TIME_SECONDS = 0.08;
    private static final double MAX_YAW_ACCEL_RAD_PER_SEC_SQ = 20.0;
    private static final int HEIGHT_COUPLING_ITERATIONS = 5;

    private final TurretSubsystem turretSubsystem;
    private final GyroSubsystem gyroSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final SmartCameraNetwork cameraNetwork;
    private final SlewRateLimiter angleStepLimiter;
    private double previousOmegaRadPerSec = 0.0;
    private double previousTimestampSec = 0.0;
    private boolean hasPreviousOmega = false;
    
    public TurretLockCommand(TurretSubsystem turretSubsystem, GyroSubsystem gyroSubsystem, CommandSwerveDrivetrain drivetrain, SmartCameraNetwork cameraNetwork) {
        this.turretSubsystem = turretSubsystem;
        this.gyroSubsystem = gyroSubsystem;
        this.cameraNetwork = cameraNetwork;
        this.drivetrain = drivetrain;
        this.angleStepLimiter = new SlewRateLimiter(Constants.Turret.Horizontal.MAX_TRACK_RATE_DEGREES_PER_SEC);
        addRequirements(turretSubsystem, gyroSubsystem);
    }

    @Override
    public void initialize() {
        angleStepLimiter.reset(0.0);
        hasPreviousOmega = false;
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

        // Calculate center of HUB
        TargetObservation bestObservation = observations.get(bestID);
        Translation2d hubOffset = Constants.RelativeHubLocation.MY_APRIL_POS.get(bestID);
        double xOffset = Units.inchesToMeters(hubOffset.getX());
        double yOffset = Units.inchesToMeters(hubOffset.getY());
        Transform3d robotToTarget = bestObservation.getRobotToTarget();

        // Calculate angle and speed at which to shoot the ball at
        var s = this.drivetrain.getState().Speeds;
        double vxRobot = s.vxMetersPerSecond;
        double vyRobot = s.vyMetersPerSecond;
        double omega = s.omegaRadiansPerSecond;
        double speedMagnitude = Constants.Turret.MAX_SPEED_MEASURED_MpS;

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

        // Predict yaw rate at launch and include turret tangential velocity from rotation.
        double omegaAtLaunch = omega + yawAccel * LAUNCH_LEAD_TIME_SECONDS;
        double turretVelocityX = vxRobot - omegaAtLaunch * Constants.Turret.Offset.Y;
        double turretVelocityY = vyRobot + omegaAtLaunch * Constants.Turret.Offset.X;

        double phi = 0.0;
        double launchAngleDegrees = Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES;
        OptionalDouble flightTime = OptionalDouble.empty();

        for (int i = 0; i < HEIGHT_COUPLING_ITERATIONS; i++) {
            double releaseHeightMeters = Constants.Robot.ROBOT_HEIGHT_WITH_TURRET.apply(
                Units.degreesToRadians(launchAngleDegrees)
            );
            double zOffset = Units.inchesToMeters(Constants.RelativeHubLocation.Z_POS) - releaseHeightMeters;

            Transform3d robotToOpening = robotToTarget.plus(new Transform3d(
                new Translation3d(xOffset, yOffset, zOffset),
                new Rotation3d()
            ));
            Transform3d turretToOpening = robotToOpening.plus(new Transform3d(
                new Translation3d(Constants.Turret.Offset.X, Constants.Turret.Offset.Y, 0.0),
                new Rotation3d()
            ));

            double x = turretToOpening.getX();
            double y = turretToOpening.getY();
            double z = turretToOpening.getZ();

            flightTime = solveFlightTimeSeconds(x, y, z, turretVelocityX, turretVelocityY, speedMagnitude);
            if (flightTime.isEmpty()) {
                turretSubsystem.setHorizontalMotor(0.0);
                angleStepLimiter.reset(0.0);
                return;
            }

            double t = flightTime.getAsDouble();
            double shotVx = (x - turretVelocityX * t) / t;
            double shotVy = (y - turretVelocityY * t) / t;
            double shotVz = (z + 0.5 * GRAVITY_METERS_PER_SEC_SQ * t * t) / t;

            phi = -Math.toDegrees(Math.atan2(shotVy, shotVx));
            launchAngleDegrees = MathUtil.clamp(
                Math.toDegrees(Math.atan2(shotVz, Math.hypot(shotVx, shotVy))),
                Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES,
                Constants.Turret.Vertical.MAX_LAUNCH_ANGLE_DEGREES
            );
        }

        double desiredStep = 0.0;
        if (Math.abs(phi) > Constants.Vision.ANGLE_DEADBAND) {
            desiredStep = MathUtil.clamp(
                phi * Constants.Turret.Horizontal.ANGLE_TURN_PERCENTAGE,
                -Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES,
                Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES
            );
        }
        turretSubsystem.turnHorizontalMotor(angleStepLimiter.calculate(desiredStep));
        turretSubsystem.setVerticalMotor(launchAngleDegrees);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.stopHorizontal();
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
