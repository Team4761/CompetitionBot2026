package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendCommandCAUGHT extends Command {
    private enum ExtendSequenceStep {
        ENABLE_COAST,
        DISABLE_COAST,
        CATCH,
        REALEASE,
        COMPLETE
    }

    private IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();
    private ExtendSequenceStep step = ExtendSequenceStep.COMPLETE;

    public ExtendCommandCAUGHT(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
        addRequirements(sub);
    }
    
    public void initialize(){
        timer.restart();
        step = ExtendSequenceStep.ENABLE_COAST;
        System.out.println("TURNING");
        this.intakeSubsystem.turnExtenderMotorAngle(-30);//Start the intake falling
    }
//The premis is to start the intake falling via gravity then catch it bu running the motor in the opposite dirction
    public void execute() {
        switch (step) {
            case ENABLE_COAST:
                if (timer.hasElapsed(0.2)) {
                    this.intakeSubsystem.enableExtenderCoasting();//let fall
                    step = ExtendSequenceStep.DISABLE_COAST;
                }
                break;
            case DISABLE_COAST:
                if (timer.hasElapsed(1)) {
                    this.intakeSubsystem.disableExtenderCoasting();
                    step = ExtendSequenceStep.CATCH;//start stopping the fall
                }
                break;
            
            case CATCH:
                if (timer.hasElapsed(1.1)) {
                    this.intakeSubsystem.runExtenderMotor(-1);//catch at the bottom
                    step = ExtendSequenceStep.REALEASE;
                }
                break;
            case REALEASE:
                if (timer.hasElapsed(1.3)) {
                    this.intakeSubsystem.runExtenderMotor(0);
                    step = ExtendSequenceStep.COMPLETE;//stop the catch
                }
                break;
            case COMPLETE:
                break;//done
        }
    }

    public boolean isFinished() {
        return timer.hasElapsed(2.0);
    }

    public void end(boolean isInterrupted){
        timer.stop();
        intakeSubsystem.stopExtenderMotor();
    }

}
