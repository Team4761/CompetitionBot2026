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
            .build();
        this.kickerMotor = SmartVortexMotor.Builder.newInstance()
            .port(Constants.Turret.KICKER_MOTOR_PORT)
            .build();

        this.spitterMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.SPITTER_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(-1, -1)
            .mode(SmartKrakenMotor.MotorMode.WRAPPED)
            .build();
        this.horizontalMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.HORIZONTAL_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(Constants.Turret.Horizontal.ANGLE_LIM_LEFT * Constants.Turret.Horizontal.CONVERSION_FACTOR_TtoM, 
                        Constants.Turret.Horizontal.ANGLE_LIM_RIGHT * Constants.Turret.Horizontal.CONVERSION_FACTOR_TtoM)
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .build();
        this.verticalMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.VERTICAL_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(Constants.Turret.Vertical.ANGLE_LIM_LEFT * Constants.Turret.Vertical.CONVERSION_FACTOR_MtoH,
                        Constants.Turret.Vertical.ANGLE_LIM_RIGHT * Constants.Turret.Vertical.CONVERSION_FACTOR_MtoH)
            .mode(SmartKrakenMotor.MotorMode.WRAPPED)
            .build();
    }

    public void setKickerMotorSpeed(double speed) { kickerMotor.setSpeedPercent(speed); }
    public void setSpitterMotorSpeed(double speed) { spitterMotor.setSpeedPercent(speed); }
    public void setSpindexerMotorSpeed(double speed) { spindexerMotor.setSpeedPercent(speed); }

    public void stopKicker() { kickerMotor.stopTurning(); }
    public void stopSpitter() { spitterMotor.stopTurning(); }
    public void stopSpindexer() { spindexerMotor.stopTurning(); }

    public void turnHorizontalMotor(double degrees) { 
        if (!horizontalMotor.turn(degrees)) {
            horizontalMotor.set(0);
        } 
    }
    public void setHorizontalMotor(double degrees) { horizontalMotor.set(degrees); }
    public void turnVerticalMotor(double degrees) { verticalMotor.turn(degrees); }
    public void setVerticalMotor(double degrees) { verticalMotor.set(degrees * Constants.Turret.Vertical.CONVERSION_FACTOR_MtoH); }

    public void stopHorizontal() { horizontalMotor.stopTurning(); }
    public void stopVertical() { verticalMotor.stopTurning(); }
}