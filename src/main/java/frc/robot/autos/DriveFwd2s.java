package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveFwd2s extends SequentialCommandGroup {
    public DriveFwd2s(SwerveSubsystem swerve) {
        addCommands(
                swerve.reZeroCommand(),
                swerve.driveCommand(() -> 0.35, () -> 0.35, () -> 1).withTimeout(2.0),
                swerve.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0).withTimeout(0.1)
            );
    }
}
