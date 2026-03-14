package frc.robot.basecommands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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

    private final SwerveRequest.FieldCentric moveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private Pose2d targetPose = Pose2d.kZero;

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
        Pose2d startPose = drivetrain.getState().Pose;
        Translation2d fieldRelativeTranslation =
            new Translation2d(deltaXMeters, deltaYMeters).rotateBy(startPose.getRotation());

        targetPose = new Pose2d(
            startPose.getTranslation().plus(fieldRelativeTranslation),
            startPose.getRotation().plus(Rotation2d.fromDegrees(deltaDegrees))
        );

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        double xVelocityMps = MathUtil.clamp(
            xController.calculate(currentPose.getX(), targetPose.getX()),
            -Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS,
            Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS
        );
        double yVelocityMps = MathUtil.clamp(
            yController.calculate(currentPose.getY(), targetPose.getY()),
            -Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS,
            Constants.Swerve.Auto.MAX_TRANSLATION_SPEED_MPS
        );
        double rotationalRateRadPerSec = MathUtil.clamp(
            thetaController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            ),
            -Constants.Swerve.Auto.MAX_ROTATION_SPEED_RAD_PER_SEC,
            Constants.Swerve.Auto.MAX_ROTATION_SPEED_RAD_PER_SEC
        );

        drivetrain.setControl(
            moveRequest.withVelocityX(xVelocityMps)
                .withVelocityY(yVelocityMps)
                .withRotationalRate(rotationalRateRadPerSec)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(idleRequest);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
