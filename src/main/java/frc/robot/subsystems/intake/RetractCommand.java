package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class RetractCommand extends Command{
    private double retractSpeed;
    private IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();

    /**
     * 
     * @param sub is the subsystem the intake subsystem
     * @param speed speed sets the speed to the input
     */
    public RetractCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
    }

    @Override
    public void initialize() {
        timer.restart();
        intakeSubsystem.runExtenderMotor(-1 * Constants.Intake.RETRACTION_SPEED);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.Intake.SECONDS_IT_TAKES_TO_RETRACT_AT_FULL_SPEED);
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopExtenderMotor();
    }
}
