package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class RetractCommand extends Command{
    private double retractSpeed;
    private IntakeSubsystem intakeSubsystem;

    /**
     * 
     * @param sub is the subsystem the intake subsystem
     * @param speed speed sets the speed to the input
     */
    public RetractCommand(IntakeSubsystem sub, double speed) {
        this.retractSpeed = speed;
        this.intakeSubsystem = sub;
    }

    @Override
    public void initialize() {
        intakeSubsystem.runExtenderMotor(-1 * retractSpeed);
    }

    @Override
    public boolean isFinished() {
        //intakeSubsystem.stopExtenderMotor();
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopExtenderMotor();
    }
}
