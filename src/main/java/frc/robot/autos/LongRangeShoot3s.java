package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.turret.ShootAtAngleAtSpeedCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class LongRangeShoot3s extends SequentialCommandGroup{
    
    public LongRangeShoot3s(TurretSubsystem turretSubsystem) {
    
        addCommands(

            new ShootAtAngleAtSpeedCommand(turretSubsystem, 0, 1).withTimeout(4)

        ); 
    }
}
