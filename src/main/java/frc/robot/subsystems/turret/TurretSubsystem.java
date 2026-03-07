package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
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
            .PID(0.2, 0.0, 0.001)
            .outputRange(-1, 1)
            .angleLimits(Constants.Turret.Horizontal.ANGLE_LIM_LEFT * Constants.Turret.Horizontal.CONVERSION_FACTOR_MtoT, 
                        Constants.Turret.Horizontal.ANGLE_LIM_RIGHT * Constants.Turret.Horizontal.CONVERSION_FACTOR_MtoT)
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .build();
        this.verticalMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.VERTICAL_MOTOR_PORT)
            .PID(0.2, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(
                Constants.Turret.Vertical.MIN_HOOD_ANGLE_DEGREES * Constants.Turret.Vertical.CONVERSION_FACTOR_HtoM,
                Constants.Turret.Vertical.MAX_HOOD_ANGLE_DEGREES * Constants.Turret.Vertical.CONVERSION_FACTOR_HtoM
            )
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .build();
    }

    public void setKickerMotorSpeed(double speed) { kickerMotor.setSpeedPercent(speed); }
    public void setSpitterMotorSpeed(double speed) { spitterMotor.setSpeedPercent(speed); }
    public void setSpindexerMotorSpeed(double speed) { spindexerMotor.setSpeedPercent(speed); }

    public void stopKicker() { kickerMotor.stopTurning(); }
    public void stopSpitter() { spitterMotor.stopTurning(); }
    public void stopSpindexer() { spindexerMotor.stopTurning(); }

    public void turnHorizontalMotor(double degrees) { 
        double motorDegrees = toHorizontalMotorDegrees(degrees);
        if (!horizontalMotor.turn(motorDegrees)) {
            //horizontalMotor.set(0);
            System.out.println(String.format("Failed to turn horizontal motor by [%.2f] degrees. Current angle: [%.2f]", degrees, horizontalMotor.getAngle()));
        } 
    }
    public void setHorizontalMotor(double degrees) { horizontalMotor.set(toHorizontalMotorDegrees(degrees)); }
    public void turnVerticalMotor(double degrees) { verticalMotor.turn(toVerticalMotorDegrees(degrees)); }
    public void setVerticalMotor(double launchAngleDegrees) {
        verticalMotor.set(toVerticalMotorDegrees(toHoodDegreesFromLaunchAngle(launchAngleDegrees)));
    }

    public void stopHorizontal() { horizontalMotor.stopTurning(); }
    public void stopVertical() { verticalMotor.stopTurning(); }

    private double toHorizontalMotorDegrees(double turretDegrees) {
        return turretDegrees * Constants.Turret.Horizontal.CONVERSION_FACTOR_MtoT;
    }

    private double toVerticalMotorDegrees(double hoodDegrees) {
        return hoodDegrees * Constants.Turret.Vertical.CONVERSION_FACTOR_HtoM;
    }

    private double toHoodDegreesFromLaunchAngle(double launchAngleDegrees) {
        double clampedLaunchAngle = MathUtil.clamp(
            launchAngleDegrees,
            Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES,
            Constants.Turret.Vertical.MAX_LAUNCH_ANGLE_DEGREES
        );
        // The bottom hood stop already shoots upward, so launch angle is offset from hood travel.
        return clampedLaunchAngle - Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES;
    }
}
