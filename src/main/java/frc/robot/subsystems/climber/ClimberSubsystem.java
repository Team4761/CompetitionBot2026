package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartKrakenMotor.Builder;

public class ClimberSubsystem extends SubsystemBase{
    private SmartKrakenMotor climberMotor;

    public ClimberSubsystem() {
        this.climberMotor = Builder.newInstance().
            port(Constants.Climber.CLIMBER_MOTOR_PORT).
            PID(0.1, 0.0, 0.0).
            outputRange(-1, 1).
            angleLimits(-1, -1).
            build();
    }

    public void climb(double speed)
    {
        climberMotor.setSpeedPercent(speed);
    }

    public void stop()
    {
        climberMotor.setSpeedPercent(0.0);
    }

}
