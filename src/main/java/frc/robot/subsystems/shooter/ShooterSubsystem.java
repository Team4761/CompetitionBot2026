package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartVortexMotor;

public class ShooterSubsystem extends SubsystemBase {
    private final SmartVortexMotor rollerMotor;
    private final SmartVortexMotor kickerMotor;
    private final SmartKrakenMotor spitterMotor;
    
    public ShooterSubsystem() {
        this.rollerMotor = SmartVortexMotor.Builder.newInstance()
            .port(Constants.Shooter.MotorPorts.ROLLER)
            .build();
        this.kickerMotor = SmartVortexMotor.Builder.newInstance()
            .port(Constants.Shooter.MotorPorts.KICKER)
            .build();
        this.spitterMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Shooter.MotorPorts.SPITTER)
            .PID(0.1, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(-1, -1)
            .mode(SmartKrakenMotor.MotorMode.WRAPPED)
            .build();
    }

    public void setRollerMotorSpeed(double speed) { rollerMotor.setRawSpeedPercent(speed); }
    public void setKickerMotorSpeed(double speed) { kickerMotor.setRawSpeedPercent(speed); }
    public void setSpitterMotorSpeed(double speed) { spitterMotor.setRawSpeedPercent(speed); }

    public void setRollerMotorSpeedRPM(double rpm) { rollerMotor.setRawSpeed(rpm); }
    public void setKickerMotorSpeedRPM(double rpm) { kickerMotor.setRawSpeed(rpm); }
    public void setSpitterMotorSpeedRPM(double rpm) { spitterMotor.setRawSpeed(rpm); }

    public double getSpitterMotorRPM() { return spitterMotor.getSpeedRPM(); }

    public void stopKicker() { kickerMotor.stopTurning(); }
    public void stopRoller() { rollerMotor.stopTurning(); }
    public void stopSpitter() { spitterMotor.stopTurning(); }
}
