package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ExtendOrRetractCommand extends Command{

    private IntakeSubsystem intakeSubsystem;
    private double extendSpeed;


    /**
     * coomad that can extend or retract depending on if the speed is negative or positive
     * @param sub is the subsystem
     * @param speed speed at wiutch it will extend or retract 
     */
    public ExtendOrRetractCommand(IntakeSubsystem sub, double speed) {
        this.intakeSubsystem = sub;
        this.extendSpeed = speed;
    }

    public void initialize() {
        intakeSubsystem.runExtenderMotor(extendSpeed);
        new WaitCommand(Constants.Intake.SECONDS_IT_TAKES_TO_RETRACT_AT_FULL_SPEED / extendSpeed);
        intakeSubsystem.stopExtenderMotor();
    }

    public void execute() {
        
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isInterrupted){
        intakeSubsystem.stopExtenderMotor();
    }
}
