package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.baseCommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class PoseDriveExampleAuto extends SequentialCommandGroup {
    public PoseDriveExampleAuto(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new RotateRelativeDegreesCommand(drivetrain, 90.0)
        );
    }
}
