package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;

public class ClimberSubsystem extends SubsystemBase{
    private SmartKrakenMotor climberMotor;

    public ClimberSubsystem() {
        this.climberMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Intake.INTAKE_EXTENDER_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0) // Temp Values
            .outputRange(-1.0, 1.0) // Duty cycle output limits
            .angleLimits(-1, -1) // Temp Values
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .build();
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
