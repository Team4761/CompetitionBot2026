package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.STUTTERExtendCommand;

public class NEWDeployIntakeSTUTTER extends SequentialCommandGroup{

    public NEWDeployIntakeSTUTTER(IntakeSubsystem intakeSubsystem){

         
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new STUTTERExtendCommand(intakeSubsystem)
        );
        
    }
}