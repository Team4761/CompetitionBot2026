package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DoNothingCommand;
import frc.robot.subsystems.turret.*;

public class Shoot3s extends SequentialCommandGroup {
    
    public Shoot3s(TurretSubsystem turretSubsystem) {
        
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new ShootCommand(turretSubsystem).withTimeout(4)
        );
    }
}

