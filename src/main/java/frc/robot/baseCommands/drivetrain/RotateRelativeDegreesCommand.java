package frc.robot.baseCommands.drivetrain;

import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RotateRelativeDegreesCommand extends DriveToRelativePoseCommand {
    public RotateRelativeDegreesCommand(CommandSwerveDrivetrain drivetrain, double deltaDegrees) {
        super(drivetrain, 0.0, 0.0, deltaDegrees);
    }
}
