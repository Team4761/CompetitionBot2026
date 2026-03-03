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

    private double speed = (-1.0 * Constants.Turret.ShootConfig.INTAKE_SPEED_RPM);

    @Override
    public void initialize() {
        intakeSubsystem.turnIntakeMotor(-1.0 * Constants.Turret.ShootConfig.INTAKE_SPEED);
        
        //intakeSubsystem.turnIntakeMotorRPM(speed);
        System.out.println(speed);
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
