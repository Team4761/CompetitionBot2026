package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartKrakenMotor.Builder;

public class ClimberSubsystem extends SubsystemBase{
    //This is just a draft for just one of the possible versions of the climber (the rotaty one)
    //make the code aware there needs to be a motor
    private SmartKrakenMotor climberMotor;


    //there probably needs to be something to initialize the motors but idk how to do that rn
    //initilize the motor i think
    public ClimberSubsystem() {
        this.climberMotor = Builder.newInstance().
            port(Constants.Climber.CLIMBER_MOTOR_PORT).
            PID(0.1, 0.0, 0.0). // Temp Values
            outputRange(0, 360). // Temp Values
            angleLimits(-1, -1). // Temp Values
            build();
    }

    //so it can climb both up and down
    public void climb(double speed)
    {
        climberMotor.setSpeed(speed);
    }
    //stop climbing
    public void stop()
    {
        climberMotor.setSpeed(0.0);
    }

}
