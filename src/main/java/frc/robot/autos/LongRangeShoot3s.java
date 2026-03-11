package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.subsystems.turret.ShootAtAngleAtSpeedCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class LongRangeShoot3s extends SequentialCommandGroup{
    
    public LongRangeShoot3s(TurretSubsystem turretSubsystem) {
    
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new ShootAtAngleAtSpeedCommand(turretSubsystem, 0, 1).withTimeout(4)
        ); 
    }
}
