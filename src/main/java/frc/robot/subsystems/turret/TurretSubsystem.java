package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartVortexMotor;

public class TurretSubsystem extends SubsystemBase {

    public final SmartKrakenMotor spitterMotor;
    public final SmartKrakenMotor horizontalMotor;
    public final SmartKrakenMotor verticalMotor;
    public final SmartVortexMotor spindexerMotor;
    public final SmartKrakenMotor kickerMotor;

    private final PIDController horizontalPIDController;

    public TurretSubsystem() {
        this.spindexerMotor = SmartVortexMotor.Builder.newInstance()
            .port(Constants.Turret.SPINDEXER_MOTOR_PORT)
            .build();

        this.kickerMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.KICKER_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(-1, -1)
            .mode(SmartKrakenMotor.MotorMode.WRAPPED)
            .gearRatio(Constants.Turret.ShootConfig.KICKER_MOTOR_ROTATIONS_PER_ROTATION)
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
            .PID(0.5, 0.0, 0.001)
            .outputRange(-1, 1)
            .angleLimits(Constants.Turret.Horizontal.ANGLE_LIM_LEFT, Constants.Turret.Horizontal.ANGLE_LIM_RIGHT)
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .gearRatio(Constants.Turret.Horizontal.MOTOR_ROTATIONS_PER_TURRET_ROTATION)
            .build();
        this.verticalMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Turret.VERTICAL_MOTOR_PORT)
            .PID(0.5, 0.0, 0.0)
            .outputRange(-1, 1)
            .angleLimits(
                Constants.Turret.Vertical.MIN_HOOD_ANGLE_DEGREES,
                Constants.Turret.Vertical.MAX_HOOD_ANGLE_DEGREES
            )
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .gearRatio(Constants.Turret.Vertical.MOTOR_ROTATIONS_PER_HOOD_ROTATION)
            .build();

        this.horizontalPIDController = new PIDController(0.5, 0.0, 0.001);
    }
    public void setLaunchAngleDegrees(double launchAngleDegrees) {
        verticalMotor.set(getHoodAngleForLaunchAngleDegrees(launchAngleDegrees));
    }

    public double getHoodAngleForLaunchAngleDegrees(double launchAngleDegrees) {
        return toHoodDegreesFromBaselineLaunchAngle(launchAngleDegrees);
    }

    public void stepHorizontalMotor(double targetDegrees) {
        double clampedTargetDegrees = MathUtil.clamp(
            targetDegrees,
            Constants.Turret.Horizontal.ANGLE_LIM_LEFT,
            Constants.Turret.Horizontal.ANGLE_LIM_RIGHT
        );
        double currentDegrees = horizontalMotor.getAngle();
        double stepDegrees = horizontalPIDController.calculate(currentDegrees, clampedTargetDegrees);
        stepDegrees = MathUtil.clamp(
            stepDegrees,
            -Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES,
            Constants.Turret.Horizontal.MAX_TRACK_STEP_DEGREES
        );
        horizontalMotor.turn(stepDegrees);
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

    private double toHoodDegreesFromBaselineLaunchAngle(double launchAngleDegrees) {
        double maxLaunchAngleDegrees = Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES
            + (Constants.Turret.Vertical.MAX_HOOD_ANGLE_DEGREES - Constants.Turret.Vertical.MIN_HOOD_ANGLE_DEGREES);
        double clampedLaunchAngle = MathUtil.clamp(
            launchAngleDegrees,
            Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES,
            maxLaunchAngleDegrees
        );
        // The hood starts at a 22-degree launch angle at the mechanical stop and only moves upward from there.
        return Constants.Turret.Vertical.MAX_HOOD_ANGLE_DEGREES
            - (clampedLaunchAngle - Constants.Turret.Vertical.MIN_LAUNCH_ANGLE_DEGREES);
    }
}
