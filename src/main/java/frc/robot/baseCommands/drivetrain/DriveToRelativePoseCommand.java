package frc.robot.baseCommands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class DriveToRelativePoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double deltaXMeters;
    private final double deltaYMeters;
    private final double deltaDegrees;

    private final PIDController xController =
        new PIDController(Constants.Swerve.Auto.TRANSLATION_KP, 0.0, 0.0);
    private final PIDController yController =
        new PIDController(Constants.Swerve.Auto.TRANSLATION_KP, 0.0, 0.0);
    private final PIDController thetaController =
        new PIDController(Constants.Swerve.Auto.ROTATION_KP, 0.0, 0.0);

    private final SwerveRequest.RobotCentric moveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private final Timer timeoutTimer = new Timer();

    private Translation2d measuredTranslationMeters = new Translation2d();
    private Rotation2d startHeading = Rotation2d.kZero;
    private double lastTimestampSeconds = 0.0;
    private double maxDurationSeconds = 0.0;

    /**
     * 
     * @param drivetrain subsystem
     * @param deltaXMeters FORWARDBACKWARD
     * @param deltaYMeters LEFTRIGHT
     * @param deltaDegrees - is clockwise, + is counter-clockwise
     */
    public DriveToRelativePoseCommand(
        CommandSwerveDrivetrain drivetrain,
        double deltaXMeters,
        double deltaYMeters,
        double deltaDegrees
    ) {
        this.drivetrain = drivetrain;
        this.deltaXMeters = deltaXMeters;
        this.deltaYMeters = deltaYMeters;
        this.deltaDegrees = deltaDegrees;

        xController.setTolerance(Constants.Swerve.Auto.POSITION_TOLERANCE_METERS);
        yController.setTolerance(Constants.Swerve.Auto.POSITION_TOLERANCE_METERS);
        thetaController.setTolerance(Math.toRadians(Constants.Swerve.Auto.ANGLE_TOLERANCE_DEGREES));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        measuredTranslationMeters = new Translation2d();
        startHeading = drivetrain.getGyroHeading();
        lastTimestampSeconds = Timer.getFPGATimestamp();
        maxDurationSeconds = calculateMaxDurationSeconds();
        timeoutTimer.restart();
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        double currentTimestampSeconds = Timer.getFPGATimestamp();
        double deltaTimeSeconds = Math.max(0.0, currentTimestampSeconds - lastTimestampSeconds);
        lastTimestampSeconds = currentTimestampSeconds;

        Rotation2d relativeHeading = drivetrain.getGyroHeading().minus(startHeading);
        ChassisSpeeds currentSpeeds = drivetrain.getState().Speeds;
        Translation2d startRelativeVelocityMetersPerSecond =
            new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                .rotateBy(relativeHeading);

        measuredTranslationMeters = measuredTranslationMeters.plus(
            new Translation2d(
                startRelativeVelocityMetersPerSecond.getX() * deltaTimeSeconds,
                startRelativeVelocityMetersPerSecond.getY() * deltaTimeSeconds
            )
        );

        double xVelocityStartFrameMps = MathUtil.clamp(
            xController.calculate(measuredTranslationMeters.getX(), deltaXMeters),
            -Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS,
            Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS
        );
        double yVelocityStartFrameMps = MathUtil.clamp(
            yController.calculate(measuredTranslationMeters.getY(), deltaYMeters),
            -Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS,
            Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS
        );
        double rotationalRateRadPerSec = MathUtil.clamp(
            thetaController.calculate(
                relativeHeading.getRadians(),
                Math.toRadians(deltaDegrees)
            ),
            -Constants.Swerve.Auto.MAX_ROTATION_SPEED_RAD_PER_SEC,
            Constants.Swerve.Auto.MAX_ROTATION_SPEED_RAD_PER_SEC
        );

        Translation2d robotRelativeVelocityMetersPerSecond =
            new Translation2d(xVelocityStartFrameMps, yVelocityStartFrameMps)
                .rotateBy(relativeHeading.unaryMinus());

        drivetrain.setControl(
            moveRequest.withVelocityX(robotRelativeVelocityMetersPerSecond.getX())
                .withVelocityY(robotRelativeVelocityMetersPerSecond.getY())
                .withRotationalRate(rotationalRateRadPerSec)
        );
    }

    @Override
    public void end(boolean interrupted) {
        timeoutTimer.stop();
        drivetrain.setControl(idleRequest);
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint())
            || timeoutTimer.hasElapsed(maxDurationSeconds);
    }

    private double calculateMaxDurationSeconds() {
        double translationSeconds =
            Math.hypot(deltaXMeters, deltaYMeters) / Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS;
        double rotationSeconds =
            Math.abs(Math.toRadians(deltaDegrees)) / Constants.Swerve.Auto.MAX_ROTATION_SPEED_RAD_PER_SEC;

        // Failsafe so a dead sensor path cannot leave the robot driving forever.
        return Math.max(0.75, (3.0 * Math.max(translationSeconds, rotationSeconds)) + 0.5);
    }
}
