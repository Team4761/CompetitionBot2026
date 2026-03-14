package frc.robot.autos.competition;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class PreloadShootAuto extends SequentialCommandGroup {
    public PreloadShootAuto(TurretSubsystem turretSubsystem) {
        addCommands(
            new DoNothingCommand(),
            Commands.runOnce(() -> turretSubsystem.setHorizontalMotor(0.0), turretSubsystem),
            // red: check if sees tag 8 9 10 or 11; blue: check if sees tag 24 25 26 or 27
            new ShootCommand(turretSubsystem).withTimeout(4.0)
        );
    }
}
