package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtendCommand extends Command{
    private double extendSpeed;
    private IntakeSubsystem intakeSubsystem;

    /**
     * 
     * @param sub is the subsystem the intake subsystem
     * @param speed speed sets the speed to the input
     */
    public ExtendCommand(IntakeSubsystem sub, double speed) {
        this.extendSpeed = speed;
        this.intakeSubsystem = sub;
    }

    //run the extender motor so that it extends
    @Override
    public void initialize() {
        intakeSubsystem.runExtenderMotor(extendSpeed);
    }

    //stop the motor when its done idk if neccesary
    @Override
    public boolean isFinished() {
        intakeSubsystem.stopExtenderMotor();
        return true;
    }

    //stop the motor if it is interrupted idk if neccesary
    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopExtenderMotor();
    }

}
