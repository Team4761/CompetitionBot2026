package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartVortexMotor;

public class ArmSubsystem extends SubsystemBase {
    public final SmartKrakenMotor shoulderMotor;
    public final SmartKrakenMotor elbowMotor;

    public ArmSubsystem() {
        this.shoulderMotor = SmartKrakenMotor.Builder.newInstance()
        .port(Constants.Arm.SHOULDER_MOTOR_PORT)
        .PID(0.1, 0.0, 0.0)
        .outputRange(-1, 1)
        .angleLimits(Constants.Arm.MIN_SHOULDER_ANGLE, Constants.Arm.MAX_SHOULDER_ANGLE)
        .mode(SmartKrakenMotor.MotorMode.WRAPPED)
        .gearRatio(2.0)
        .build();
        this.elbowMotor = SmartKrakenMotor.Builder.newInstance()
        .port(Constants.Arm.ELBOW_MOTOR_PORT)
        .PID(0.1, 0.0, 0.0)
        .outputRange(-1, 1)
        .angleLimits(Constants.Arm.MIN_ELBOW_ANGLE, Constants.Arm.MAX_ELBOW_ANGLE)
        .mode(SmartKrakenMotor.MotorMode.WRAPPED)
        .gearRatio(2.0)
        .build();
    }
    
    public void setElbowAngleDegrees(double elbowAngleDegrees) {
        elbowMotor.set(elbowAngleDegrees);
    }

    public void setShoulderAngleDegrees(double shoulderAngleDegrees) {
        shoulderMotor.set(shoulderAngleDegrees);
    }    
}
