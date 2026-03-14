package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class STUTTERExtendCommand extends Command {
    private enum ExtendSequenceStep {
        TURN_CHUNK,
        COMPLETE
    }

    private IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();
    private ExtendSequenceStep step = ExtendSequenceStep.COMPLETE;

    public STUTTERExtendCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
        addRequirements(sub);
    }
    
    public void initialize(){
        timer.restart();
        step = ExtendSequenceStep.TURN_CHUNK;
        System.out.println("TURNING");
        this.intakeSubsystem.turnExtenderMotorAngle(-15);
    }

    public void execute() {
        switch (step) {
            case TURN_CHUNK:
                if (timer.get() % 0.2 < 0.2 && !(this.intakeSubsystem.getExtenderMotorAngle() > 95)) {
                    this.intakeSubsystem.turnExtenderMotorAngle(-15);
                } else if (this.intakeSubsystem.getExtenderMotorAngle() > 95.0) {
                    step = ExtendSequenceStep.COMPLETE;
                }
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
