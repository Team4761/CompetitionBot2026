package frc.robot.subsystems.turret.commands;

import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootCommand extends ShootWithPowerCommand {
    public ShootCommand(TurretSubsystem sub) {
        super(sub, Constants.Turret.ShootConfig.SPITTER_SPEED);
    }
}
