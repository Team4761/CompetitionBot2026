package frc.robot.autos;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurnMotors extends SequentialCommandGroup {
    private static final double AUTO_TURN_DEGREES = Units.radiansToDegrees(
        0.35 * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 2.0
    );

    public TurnMotors(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new RotateRelativeDegreesCommand(drivetrain, AUTO_TURN_DEGREES)
        );
    }
}
