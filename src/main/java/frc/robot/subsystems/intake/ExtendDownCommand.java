package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtendDownCommand extends Command {
    
    private IntakeSubsystem intakeSubsystem;

    public ExtendDownCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;

    }
    /* 
    public void initialize(){
        intakeSubsystem.setExtenderMotorAngle(95);
        intakeSubsystem.coastExtenderMotor();
        Timer.delay(0.1);
        intakeSubsystem.runExtenderMotor(-0.1);
        intakeSubsystem.brakeExtenderMotor();

    }*/

    public void execute() {

    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean isInterrupted){
        intakeSubsystem.stopExtenderMotor();
    }

}
