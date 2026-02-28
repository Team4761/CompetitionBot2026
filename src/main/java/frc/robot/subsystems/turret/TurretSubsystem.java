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
        double verticalMinAngle = Math.min(Constants.Turret.Vertical.ANGLE_LIM_LEFT, Constants.Turret.Vertical.ANGLE_LIM_RIGHT);
        double verticalMaxAngle = Math.max(Constants.Turret.Vertical.ANGLE_LIM_LEFT, Constants.Turret.Vertical.ANGLE_LIM_RIGHT);

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
            .angleLimits(Constants.Turret.Horizontal.ANGLE_LIM_LEFT, Constants.Turret.Horizontal.ANGLE_LIM_RIGHT)
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .motorRotationsPerMechanismRotation(Constants.Turret.Horizontal.CONVERSION_FACTOR_TtoM)
            .build();
        this.verticalMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.VERTICAL_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(verticalMinAngle, verticalMaxAngle)
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .motorRotationsPerMechanismRotation(Constants.Turret.Vertical.CONVERSION_FACTOR_HtoM)
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
            horizontalMotor.stopTurning();
        } 
    }
    public void setHorizontalMotor(double degrees) {
        if (!horizontalMotor.set(degrees)) {
            horizontalMotor.stopTurning();
        }
    }
    public void turnVerticalMotor(double degrees) {
        if (!verticalMotor.turn(degrees)) {
            verticalMotor.stopTurning();
        }
    }
    public void setVerticalMotor(double degrees) {
        if (!verticalMotor.set(degrees)) {
            verticalMotor.stopTurning();
        }
    }

    public void stopHorizontal() { horizontalMotor.stopTurning(); }
    public void stopVertical() { verticalMotor.stopTurning(); }
}
