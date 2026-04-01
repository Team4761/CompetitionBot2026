package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootAtAngleCommand extends SequentialCommandGroup {
    public ShootAtAngleCommand(TurretSubsystem turret, double angle) {
        addCommands(
                new TurretAimChangeCommand(turret, () -> 0.0, () -> angle),
                new ShootCommand(turret)
            );
    }
}
