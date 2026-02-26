package frc.robot.autos;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurnMotors extends SequentialCommandGroup {
    private static final double AUTO_TURN_RATE_RAD_PER_SEC =
        0.35 * RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public TurnMotors(CommandSwerveDrivetrain drivetrain) {
        final var idle = new SwerveRequest.Idle();

        addCommands(
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(AUTO_TURN_RATE_RAD_PER_SEC)
            ).withTimeout(2.0),
            drivetrain.applyRequest(() -> idle).withTimeout(0.1)
        );
    }
}
