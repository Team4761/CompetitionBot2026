package frc.robot.subsystems.swerve.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SnapToHubCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xVelocitySupplier;
    private final DoubleSupplier yVelocitySupplier;
    private final double maxAngularRate;
    private final PIDController headingController =
        new PIDController(Constants.Swerve.Auto.ROTATION_KP, 0.0, 0.0);
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public SnapToHubCommand(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier xVelocitySupplier,
        DoubleSupplier yVelocitySupplier,
        double maxAngularRate
    ) {
        this.drivetrain = drivetrain;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.maxAngularRate = maxAngularRate;

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingController.reset();
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = drivetrain.getEstimatedPose().getRotation();
        Rotation2d targetHeading = drivetrain.getHubAimHeading();
        double omega = MathUtil.clamp(
            headingController.calculate(currentHeading.getRadians(), targetHeading.getRadians()),
            -maxAngularRate,
            maxAngularRate
        );

        drivetrain.setControl(
            driveRequest.withVelocityX(xVelocitySupplier.getAsDouble())
                .withVelocityY(yVelocitySupplier.getAsDouble())
                .withRotationalRate(omega)
        );
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        headingController.reset();
    }
}
