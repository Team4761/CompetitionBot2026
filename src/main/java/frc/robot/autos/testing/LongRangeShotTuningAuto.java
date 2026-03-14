package frc.robot.autos.testing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class LongRangeShotTuningAuto extends SequentialCommandGroup {
    public LongRangeShotTuningAuto(TurretSubsystem turretSubsystem) {
        addCommands(
            new DoNothingCommand(),
            // red: check if sees tag 8 9 10 or 11; blue: check if sees tag 24 25 26 or 27
            // new ShootAtAngleAtSpeedCommand(turretSubsystem, 0, 1).withTimeout(4)
            new ShootCommand(turretSubsystem).withTimeout(4.0)
        );
    }
}
