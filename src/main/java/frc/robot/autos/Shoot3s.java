package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.turret.*;
import frc.robot.subsystems.turret.ShootCommand;

public class Shoot3s extends SequentialCommandGroup {
    
    public Shoot3s(TurretSubsystem turretSubsystem) {
        
        addCommands(
            new ShootCommand(turretSubsystem).withTimeout(4)
        );
    }
}

