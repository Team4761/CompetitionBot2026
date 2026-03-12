package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class JostleCommand extends Command{
    private IntakeSubsystem intakeSubsystem;
    private enum ExtendSequenceStep {
        LIFT_UP,
        LET_FALL,
        REPEAT,
        COMPLETE
    }
    private Double Reapeats = 0.0;

    private final Timer timer = new Timer();
    private ExtendSequenceStep step = ExtendSequenceStep.COMPLETE;
    /**
     * 
     * @param sub is the subsystem the intake subsystem
     */
    public JostleCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        timer.reset();
        step = ExtendSequenceStep.LIFT_UP;
    }

    @Override
    public void execute() {
        intakeSubsystem.runExtenderMotor(-1);
        new WaitCommand(1);
        intakeSubsystem.stopExtenderMotor();
        new WaitCommand(1);
        switch (step) {
            case LIFT_UP:
                if (timer.hasElapsed(1.0 + this.Reapeats)) {
                    this.intakeSubsystem.turnExtenderMotorAngle(-20);
                    step = ExtendSequenceStep.LET_FALL;
                }
                break;
            case LET_FALL:
                if (timer.hasElapsed(3.0 + this.Reapeats)) {
                    this.intakeSubsystem.turnExtenderMotorAngle(-20);
                    step = ExtendSequenceStep.REPEAT;
                }
                break;
            case REPEAT:
                if (timer.hasElapsed(5.0 + this.Reapeats)) {
                    step = ExtendSequenceStep.LIFT_UP;
                    this.Reapeats += 5.0;
                }
                if (timer.hasElapsed(20.0))
                {
                    step = ExtendSequenceStep.COMPLETE;
                }
                break;
            case COMPLETE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(24.0);
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopExtenderMotor();
    }
}
