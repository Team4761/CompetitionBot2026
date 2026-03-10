package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendCommand extends Command {
    private static final double SECOND_EXTENSION_TIME_SECONDS = 0.2;
    private static final double DISABLE_COASTING_TIME_SECONDS = 0.3;
    private static final double ENABLE_COASTING_TIME_SECONDS = 0.4;

    private final IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();
    private int step;

    public ExtendCommand(IntakeSubsystem sub) {
        this.intakeSubsystem = sub;
        addRequirements(sub);
    }
    
    public void initialize(){
        step = 0;
        timer.restart();
        System.out.println("TURNING");
        this.intakeSubsystem.turnExtenderMotorAngle(-30);
    }

    public void execute() {
        if (step == 0 && timer.hasElapsed(SECOND_EXTENSION_TIME_SECONDS)) {
            this.intakeSubsystem.turnExtenderMotorAngle(-30);
            step = 1;
        } else if (step == 1 && timer.hasElapsed(DISABLE_COASTING_TIME_SECONDS)) {
            this.intakeSubsystem.disableExtenderCoasting();
            step = 2;
        } else if (step == 2 && timer.hasElapsed(ENABLE_COASTING_TIME_SECONDS)) {
            this.intakeSubsystem.enableExtenderCoasting();
            step = 3;
        }
    }

    public boolean isFinished() {
        return timer.hasElapsed(2.0);
    }

    public void end(boolean isInterrupted){
        intakeSubsystem.stopExtenderMotor();
    }

}
