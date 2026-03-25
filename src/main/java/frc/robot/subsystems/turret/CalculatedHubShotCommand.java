package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class CalculatedHubShotCommand extends Command {
    private static final double GRAVITY_MPS2 = 9.81;
    private static final double BALL_SPEED_AT_MAX_RPM_MPS = 13.0;
    private static final double MIN_LAUNCH_ANGLE_DEGREES = Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES;
    // Hood starts at 22 degrees and can only increase from there.
    private static final double MAX_LAUNCH_ANGLE_DEGREES = MIN_LAUNCH_ANGLE_DEGREES
        + (Constants.Turret.Vertical.MAX_HOOD_ANGLE_DEGREES - Constants.Turret.Vertical.MIN_HOOD_ANGLE_DEGREES);
    private static final double MAX_SPITTER_RPM = Constants.Shooter.Config.MAX_SHOOTER_SPEED_RPM;
    private static final double RPM_PER_MPS = MAX_SPITTER_RPM / BALL_SPEED_AT_MAX_RPM_MPS;
    private static final double ANGLE_STEP_DEGREES = 0.25;

    private final TurretSubsystem turretSubsystem;
    private final CommandSwerveDrivetrain drivetrainSubsystem;
    private final Timer feederDelayTimer = new Timer();

    private boolean feedersStarted;
    private ShotSolution currentSolution = ShotSolution.noSolution(0.0);

    public CalculatedHubShotCommand(TurretSubsystem turretSubsystem, CommandSwerveDrivetrain drivetrainSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        feedersStarted = false;
        feederDelayTimer.restart();
        currentSolution = calculateShotSolution();
        applyShotSolution(currentSolution);
    }

    @Override
    public void execute() {
        currentSolution = calculateShotSolution();
        applyShotSolution(currentSolution);

        if (!currentSolution.feasible()) {
            feedersStarted = false;
            feederDelayTimer.restart();
            turretSubsystem.stopSpindexer();
            turretSubsystem.stopKicker();
            return;
        }

        if (!feedersStarted && feederDelayTimer.hasElapsed(Constants.Turret.ShootConfig.KICKER_INIT_DELAY)) {
            turretSubsystem.setSpindexerMotorSpeed(Constants.Turret.ShootConfig.SPINDEXER_SPEED);
            turretSubsystem.setKickerMotorSpeed(Constants.Turret.ShootConfig.KICKER_SPEED);
            feedersStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        feederDelayTimer.stop();
        turretSubsystem.stopSpitter();
        turretSubsystem.stopSpindexer();
        turretSubsystem.stopKicker();
        turretSubsystem.stopVertical();
    }

    private void applyShotSolution(ShotSolution solution) {
        SmartDashboard.putNumber("Hub Shot Distance M", solution.distanceMeters());
        SmartDashboard.putNumber("Hub Shot Launch Angle Deg", solution.launchAngleDegrees());
        SmartDashboard.putNumber("Hub Shot Required RPM", solution.requiredRpm());
        SmartDashboard.putNumber("Hub Shot Commanded RPM", solution.commandedRpm());
        SmartDashboard.putBoolean("Hub Shot Feasible", solution.feasible());

        if (!solution.hasTrajectory()) {
            turretSubsystem.stopSpitter();
            turretSubsystem.stopVertical();
            return;
        }

        turretSubsystem.setLaunchAngleDegrees(solution.launchAngleDegrees());
        turretSubsystem.setSpitterMotorSpeedRPM(solution.commandedRpm());
    }

    private ShotSolution calculateShotSolution() {
        double distanceMeters = drivetrainSubsystem.getDistanceToHubMeters();
        if (!Double.isFinite(distanceMeters) || distanceMeters <= 0.0) {
            return ShotSolution.noSolution(distanceMeters);
        }

        ShotSolution bestSolution = null;
        for (double launchAngleDegrees = MIN_LAUNCH_ANGLE_DEGREES;
             launchAngleDegrees <= MAX_LAUNCH_ANGLE_DEGREES + 1e-9;
             launchAngleDegrees += ANGLE_STEP_DEGREES) {
            double launchAngleRadians = Math.toRadians(launchAngleDegrees);
            double launchHeightMeters = Constants.Robot.ROBOT_HEIGHT_WITH_TURRET.apply(launchAngleDegrees);
            double heightDeltaMeters = FieldConstants.Hub.HUB_HEIGHT - launchHeightMeters;
            double denominator = 2.0 * Math.pow(Math.cos(launchAngleRadians), 2.0)
                * ((distanceMeters * Math.tan(launchAngleRadians)) - heightDeltaMeters);

            if (denominator <= 0.0) {
                continue;
            }

            double requiredSpeedSquared = (GRAVITY_MPS2 * distanceMeters * distanceMeters) / denominator;
            if (!Double.isFinite(requiredSpeedSquared) || requiredSpeedSquared <= 0.0) {
                continue;
            }

            double requiredRpm = Math.sqrt(requiredSpeedSquared) * RPM_PER_MPS;
            ShotSolution candidate = new ShotSolution(
                distanceMeters,
                launchAngleDegrees,
                requiredRpm,
                MathUtil.clamp(requiredRpm, 0.0, MAX_SPITTER_RPM),
                requiredRpm <= MAX_SPITTER_RPM,
                true
            );

            if (bestSolution == null
                || (candidate.feasible() && !bestSolution.feasible())
                || (candidate.feasible() == bestSolution.feasible()
                    && candidate.requiredRpm() < bestSolution.requiredRpm())) {
                bestSolution = candidate;
            }
        }

        return bestSolution != null ? bestSolution : ShotSolution.noSolution(distanceMeters);
    }

    private static record ShotSolution(
        double distanceMeters,
        double launchAngleDegrees,
        double requiredRpm,
        double commandedRpm,
        boolean feasible,
        boolean hasTrajectory
    ) {
        private static ShotSolution noSolution(double distanceMeters) {
            return new ShotSolution(
                distanceMeters,
                MIN_LAUNCH_ANGLE_DEGREES,
                0.0,
                0.0,
                false,
                false
            );
        }
    }
}
