package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command{
    private double intakeSpeed;
    private IntakeSubsystem intakeSubsystem;

    /**
     * 
     * @param sub is the subsystem the intake subsystem
     * @param speed speed sets the speed to the input
     */
    public IntakeCommand(IntakeSubsystem sub, double speed) {
        this.intakeSpeed = speed;
        this.intakeSubsystem = sub;
    }

    @Override
    public void initialize() {
        
    }

    //run the motor contiunously so that you can intake
    @Override
    public void execute() {
        intakeSubsystem.turnIntakeMotor(intakeSpeed);
    }

    //stop the motor when it is done idk if necceary
    @Override
    public boolean isFinished() {
        intakeSubsystem.stopIntakeMotor();
        return false;
    }

    //stop the motor when it is interrupted idk if necceary
    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopIntakeMotor();
    }
}
