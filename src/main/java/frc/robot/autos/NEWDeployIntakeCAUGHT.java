package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.subsystems.intake.ExtendCommandCAUGHT;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class NEWDeployIntakeCAUGHT extends SequentialCommandGroup{

    public NEWDeployIntakeCAUGHT(IntakeSubsystem intakeSubsystem){

         
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new ExtendCommandCAUGHT(intakeSubsystem)
        );
        
    }
}