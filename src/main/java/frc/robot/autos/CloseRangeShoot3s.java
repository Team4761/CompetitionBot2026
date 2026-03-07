package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DoNothingCommand;
import frc.robot.subsystems.turret.ShootAtAngleAtSpeedCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class CloseRangeShoot3s extends SequentialCommandGroup{
    
     public CloseRangeShoot3s(TurretSubsystem turretSubsystem) {
        
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new ShootAtAngleAtSpeedCommand(turretSubsystem, -31, 0.5).withTimeout(4)  // angle is tbd as the angle 
        ); 
    }
}
