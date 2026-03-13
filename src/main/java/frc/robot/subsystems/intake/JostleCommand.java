package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class JostleCommand extends Command{
    private IntakeSubsystem intakeSubsystem;

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
    }

    @Override
    public void execute() {
        intakeSubsystem.runExtenderMotor(-1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopExtenderMotor();
    }
}
