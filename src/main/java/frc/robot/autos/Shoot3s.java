package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.coreCommands.DoNothingCommand;
import frc.robot.subsystems.turret.*;

public class Shoot3s extends SequentialCommandGroup {
    
    public Shoot3s(TurretSubsystem turretSubsystem) {
        
        switch (Constants.Field.ALLIANCE_COLOR){
            case "RED": {
                addCommands(
                    new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
                    //check if sees tag 8 9 10 or 11
                    new ShootCommand(turretSubsystem).withTimeout(4)
                );
            }
            case "BLUE": {
                addCommands(
                    new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
                    //check if sees tag 24 25 26 or 27
                    new ShootCommand(turretSubsystem).withTimeout(4)
                );
            }
        }
    }
}

