package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class DriveForward extends SequentialCommandGroup {
    private static final double AUTO_DISTANCE_METERS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 2.0;

    public DriveForward(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new DriveCommand(drivetrain, 1, 0, 90)
        );
    }
}
