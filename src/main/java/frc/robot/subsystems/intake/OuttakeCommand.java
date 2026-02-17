package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeCommand extends Command{
    private double outtakeSpeed;
    private IntakeSubsystem intakeSubsystem;

    /**
     * 
     * @param sub is the subsystem the intake subsystem
     * @param speed speed sets the speed to the input
     */
    public OuttakeCommand(IntakeSubsystem sub, double speed) {
        this.outtakeSpeed = speed;
        this.intakeSubsystem = sub;
    }

    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        intakeSubsystem.turnIntakeMotor(-1 * outtakeSpeed);
    }

    @Override
    public boolean isFinished() {
        intakeSubsystem.stopIntakeMotor();
        return true;
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopIntakeMotor();
    }
}
