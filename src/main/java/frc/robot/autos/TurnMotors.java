package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TurnMotors extends SequentialCommandGroup {
    public TurnMotors(SwerveSubsystem swerve) {
        addCommands(
                swerve.reZeroCommand(),
                swerve.driveCommand(() -> 0.0, () -> 0.0, () -> 0.35).withTimeout(2.0),
                swerve.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0).withTimeout(0.1)
            );
    }
}
