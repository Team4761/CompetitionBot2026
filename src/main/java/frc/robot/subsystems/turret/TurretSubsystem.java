package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartVortexMotor;

public class TurretSubsystem extends SubsystemBase {

    private final SmartKrakenMotor spitterMotor;
    private final SmartKrakenMotor horizontalMotor;
    private final SmartKrakenMotor verticalMotor;
    private final SmartVortexMotor spindexerMotor;
    private final SmartVortexMotor kickerMotor;

    public TurretSubsystem() {
        this.spindexerMotor = SmartVortexMotor.Builder.newInstance()
            .port(Constants.Turret.SPINDEXER_MOTOR_PORT)
            .angleLimits(-1, -1)
            .build();
        this.kickerMotor = SmartVortexMotor.Builder.newInstance()
            .port(Constants.Turret.KICKER_MOTOR_PORT)
            .angleLimits(-1, -1)
            .build();

        this.spitterMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.SPITTER_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(0, 360)
            .angleLimits(-1, -1)
            .build();
        this.horizontalMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.HORIZONTAL_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(0, 360)
            .angleLimits(-1, -1)
            .build();
        this.verticalMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.VERTICAL_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(0, 360)
            .angleLimits(-1, -1)
            .build();
    }

    public void setKickerMotorSpeed(double speed) { kickerMotor.setSpeed(speed); }
    public void setSpitterMotorSpeed(double speed) { spitterMotor.setSpeed(speed); }
    public void setSpindexerMotorSpeed(double speed) { spindexerMotor.setSpeed(speed); }

    public void stopKicker() { kickerMotor.stopTurning(); }
    public void stopSpitter() { spitterMotor.stopTurning(); }
    public void stopSpindexer() { spindexerMotor.stopTurning(); }

    public void turnHorizontalMotor(double degrees) { horizontalMotor.turn(degrees); }
    public void turnVerticalMotor(double degrees) { verticalMotor.turn(degrees); }

    public void stopHorizontal() { horizontalMotor.stopTurning(); }
    public void stopVertical() { verticalMotor.stopTurning(); }
}