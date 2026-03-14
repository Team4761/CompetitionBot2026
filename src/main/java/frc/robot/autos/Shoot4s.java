package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.coreCommands.DoNothingCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Shoot4s extends SequentialCommandGroup {
    public Shoot4s(TurretSubsystem turretSubsystem) {
        addCommands(
            new DoNothingCommand(),
            // red: check if sees tag 8 9 10 or 11; blue: check if sees tag 24 25 26 or 27
            new ShootCommand(turretSubsystem).withTimeout(5.0)
        );
    }
}
