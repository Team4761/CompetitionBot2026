package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveCommand;
import frc.robot.baseCommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class DriveFwd2s extends SequentialCommandGroup {
    private static final double AUTO_DISTANCE_METERS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 2.0;

    public DriveFwd2s(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new DriveCommand(drivetrain, AUTO_DISTANCE_METERS, 0.0, 0.0)
        );
    }
}
