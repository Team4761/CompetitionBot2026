package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ShootAtAngleSTUTTERCommand extends SequentialCommandGroup {
    public ShootAtAngleSTUTTERCommand(TurretSubsystem turret, double angle) {
        addCommands(
                new TurretAimChangeCommand(turret, () -> 0.0, () -> angle),
                new ShootCommandSTUTTER(turret)
            );
    }
}
