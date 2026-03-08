package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ExtendCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();

    public ExtendCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
    }
    
    public void initialize(){
        timer.restart();
        intakeSubsystem.setExtenderMotorAngle(95);
        intakeSubsystem.enableExtenderCoasting();
        Timer.delay(0.1);
        intakeSubsystem.runExtenderMotor(-0.1);
        Timer.delay(1.0);
        intakeSubsystem.disableExtenderCoasting();
    }

    public void execute() {

    }

    public boolean isFinished() {
        return timer.hasElapsed(2.0);
    }

    public void end(boolean isInterrupted){
        intakeSubsystem.stopExtenderMotor();
    }

}
