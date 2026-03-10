package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ExtendCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();

    public ExtendCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
        addRequirements(sub);
    }
    
    public void initialize(){
        timer.restart();
        System.out.println("TURNING");
        this.intakeSubsystem.turnExtenderMotorAngle(-30);
        Timer.delay(0.2);
        this.intakeSubsystem.turnExtenderMotorAngle(-30);
        Timer.delay(0.1);
        this.intakeSubsystem.disableExtenderCoasting();
        Timer.delay(0.1);
        this.intakeSubsystem.enableExtenderCoasting();
    }

    public void execute() {

    }

    public boolean isFinished() {
        return timer.hasElapsed(2.0);
    }

    public void end(boolean isInterrupted){
        intakeSubsystem.stopExtenderMotor();
    }

}
