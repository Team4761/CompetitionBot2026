package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.turret.*;

public class FireFromRoller extends SequentialCommandGroup {
    public FireFromRoller(TurretSubsystem turret) {
        addCommands(
                new SpitterSpinCommand(turret, Constants.Turret.ShootConfig.SPITTER_SPEED),
                new KickerSpinCommand(turret, Constants.Turret.ShootConfig.KICKER_SPEED),
                new RollerSpinCommand(turret, Constants.Turret.ShootConfig.ROLLER_SPEED)
            );
    }
}
