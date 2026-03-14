package frc.robot.autos.testing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.baseCommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RelativePoseDriveTestAuto extends SequentialCommandGroup {
    public RelativePoseDriveTestAuto(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new DriveRelativeMetersCommand(drivetrain, 1.0, 0.0),
            new DriveRelativeMetersCommand(drivetrain, 0.0, 0.5),
            new RotateRelativeDegreesCommand(drivetrain, 90.0)
        );
    }
}
