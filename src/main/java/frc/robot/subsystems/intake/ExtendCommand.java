package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendCommand extends Command {
    private enum ExtendSequenceStep {
        SECOND_TURN,
        DISABLE_COAST,
        ENABLE_COAST,
        COMPLETE
    }

    private IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();
    private ExtendSequenceStep step = ExtendSequenceStep.COMPLETE;

    public ExtendCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
        addRequirements(sub);
    }
    
    public void initialize(){
        timer.restart();
        step = ExtendSequenceStep.SECOND_TURN;
        System.out.println("TURNING");
        this.intakeSubsystem.turnExtenderMotorAngle(-30);
    }

    public void execute() {
        switch (step) {
            case SECOND_TURN:
                if (timer.hasElapsed(0.2)) {
                    this.intakeSubsystem.turnExtenderMotorAngle(-30);
                    step = ExtendSequenceStep.DISABLE_COAST;
                }
                break;
            case DISABLE_COAST:
                if (timer.hasElapsed(0.3)) {
                    this.intakeSubsystem.disableExtenderCoasting();
                    step = ExtendSequenceStep.ENABLE_COAST;
                }
                break;
            case ENABLE_COAST:
                if (timer.hasElapsed(0.4)) {
                    this.intakeSubsystem.enableExtenderCoasting();
                    step = ExtendSequenceStep.COMPLETE;
                }
                break;
            case COMPLETE:
                break;
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
