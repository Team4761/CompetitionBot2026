package frc.robot.coreCommands.drivetrain;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveRelativeMetersCommand extends DriveToRelativePoseCommand {
    public DriveRelativeMetersCommand(
        CommandSwerveDrivetrain drivetrain,
        double deltaXMeters,
        double deltaYMeters
    ) {
        super(drivetrain, deltaXMeters, deltaYMeters, 0.0);
    }
}
