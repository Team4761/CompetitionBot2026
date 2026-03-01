package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class OuttakeCommand extends Command{
    private IntakeSubsystem intakeSubsystem;

    /**
     * 
     * @param sub is the subsystem the intake subsystem
     */
    public OuttakeCommand(IntakeSubsystem sub) {
        
        this.intakeSubsystem = sub;
    }

    @Override
    public void initialize() {
        intakeSubsystem.turnIntakeMotor( -1 * Constants.Turret.ShootConfig.INTAKE_SPEED);
    }
    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        //intakeSubsystem.stopIntakeMotor();
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.stopIntakeMotor();
    }
}
