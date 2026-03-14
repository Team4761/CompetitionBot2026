package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.coreCommands.DoNothingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveFwd2s extends SequentialCommandGroup {
    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public DriveFwd2s(CommandSwerveDrivetrain drivetrain) {
        final var idle = new SwerveRequest.Idle();

        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(AUTO_SPEED_MPS)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(2.0),
            drivetrain.applyRequest(() -> idle).withTimeout(0.1)
        );
    }
}
