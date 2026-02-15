package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimbCommand extends Command{
    //This is just a draft for just one of the possible versions of the climber (the rotaty one)
    private double climbSpeed;
    private ClimberSubsystem climberSubsystem;
    
    
    /**
     * 
     * @param sub is the subsystem the climber subsystem
     * @param speed speed sets the speed to the input
     */
    public ClimbCommand(ClimberSubsystem sub,double speed) {
        this.climbSpeed = speed;
        this.climberSubsystem = sub;
    }

    @Override
    public void initialize() {
        climberSubsystem.climb(climbSpeed);
        new WaitCommand(5);
        climberSubsystem.climb(-climbSpeed);
    }

    @Override
    public boolean isFinished() {
        climberSubsystem.stop();
        return true;
    }

    @Override
    public void end(boolean isInterrupted) {
        climberSubsystem.stop();
    }

}
