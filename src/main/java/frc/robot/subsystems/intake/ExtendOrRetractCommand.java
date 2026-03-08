package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ExtendOrRetractCommand extends Command{

    private static final double MIN_SPEED_MAGNITUDE = 1e-3;

    private final IntakeSubsystem intakeSubsystem;
    private final double extendSpeed;
    private final Timer timer = new Timer();
    private final double moveDurationSeconds;


    /**
     * coomad that can extend or retract depending on if the speed is negative or positive
     * @param sub is the subsystem
     * @param speed speed at wiutch it will extend or retract 
     */
    public ExtendOrRetractCommand(IntakeSubsystem sub, double speed) {
        this.intakeSubsystem = sub;
        this.extendSpeed = speed;
        this.moveDurationSeconds =
            Math.abs(speed) < MIN_SPEED_MAGNITUDE
                ? 0.0
                : Constants.Intake.SECONDS_IT_TAKES_TO_RETRACT_AT_FULL_SPEED / Math.abs(speed);
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        timer.restart();
        intakeSubsystem.runExtenderMotor(extendSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return moveDurationSeconds == 0.0 || timer.hasElapsed(moveDurationSeconds);
    }

    @Override
    public void end(boolean isInterrupted){
        timer.stop();
        intakeSubsystem.stopExtenderMotor();
    }
}
