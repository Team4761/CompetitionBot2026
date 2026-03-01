package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.turret.*;

public class FireFromSpindexer extends SequentialCommandGroup {
    public FireFromSpindexer(TurretSubsystem turret) {
        addCommands(
                new SpitterSpinCommand(turret, Constants.Turret.ShootConfig.SPITTER_SPEED),
                new KickerSpinCommand(turret, Constants.Turret.ShootConfig.KICKER_SPEED),
                new SpindexSpinCommand(turret, Constants.Turret.ShootConfig.SPINDEXER_SPEED)
            );
    }
}
