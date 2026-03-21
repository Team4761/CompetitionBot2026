package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ShootAtAngleDRIFTCommand extends SequentialCommandGroup {
    public ShootAtAngleDRIFTCommand(TurretSubsystem turret, double angle) {
        addCommands(
                // https://chatgpt.com/g/g-p-696291aac8308191ae3131b700fb0636-robotics-league-2026-season/c/69b3855d-456c-832c-975e-af0275de4d5f
                new TurretAimChangeCommand(turret, () -> 0.0, () -> angle + 20.0 * (1 - (turret.getSpitterRPM() / Constants.Turret.ShootConfig.SPITTER_SPEED))),
                new ShootCommand(turret)
            );
    }
}
