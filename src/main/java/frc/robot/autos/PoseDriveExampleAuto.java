package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.baseCommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PoseDriveExampleAuto extends SequentialCommandGroup {
    public PoseDriveExampleAuto(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new DriveRelativeMetersCommand(drivetrain, 1.0, 0.0),
            new DriveRelativeMetersCommand(drivetrain, 0.0, 0.5),
            new RotateRelativeDegreesCommand(drivetrain, 90.0)
        );
    }
}
