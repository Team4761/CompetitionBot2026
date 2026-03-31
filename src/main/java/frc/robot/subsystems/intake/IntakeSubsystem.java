package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;

public class IntakeSubsystem extends SubsystemBase{
    // Make the code aware there should be 2 motors
    public final SmartKrakenMotor intakeExtenderMotor;
    public final SmartKrakenMotor intakeMotor;

    // Tell the code what those motor are/should be like and were to find them
    public IntakeSubsystem() {
        this.intakeExtenderMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Intake.INTAKE_EXTENDER_MOTOR_PORT)
            .PID(0.5, 0.0, 0.0) // Temp Values
            .outputRange(-1.0, 1.0) // Duty cycle output limits
            .angleLimits(Constants.Intake.MIN_EXTENSION_ANGLE, Constants.Intake.MAX_EXTENSION_ANGLE)
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .gearRatio(Constants.Intake.MOTOR_ROTATIONS_PER_EXTENDER_ROTATION)
            .build();
        this.intakeMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Intake.MAIN_INTAKE_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0) // Temp Values
            .outputRange(-1.0, 1.0) // Duty cycle output limits
            .angleLimits(-1, -1) // Temp Values
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .build();
    }
}
