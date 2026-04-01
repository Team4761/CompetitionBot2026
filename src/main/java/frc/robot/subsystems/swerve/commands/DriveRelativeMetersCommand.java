package frc.robot.subsystems.swerve.commands;

import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class DriveRelativeMetersCommand extends DriveToRelativePoseCommand {
    public DriveRelativeMetersCommand(
        CommandSwerveDrivetrain drivetrain,
        double deltaXMeters,
        double deltaYMeters
    ) {
        super(drivetrain, deltaXMeters, deltaYMeters, 0.0);
    }
}
