package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.basecommands.DoNothingCommand;
import frc.robot.basecommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.basecommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PoseDriveExampleAuto extends SequentialCommandGroup {
    public PoseDriveExampleAuto(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new RotateRelativeDegreesCommand(drivetrain, 90.0)
        );
    }
}
