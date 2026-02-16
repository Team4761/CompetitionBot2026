package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartKrakenMotor.Builder;

public class IntakeSubsystem extends SubsystemBase{
    //make the code aware there should be 2 motors
    private SmartKrakenMotor intakeExtenderMotor;
    private SmartKrakenMotor intakeMotor;

    //tell the code what those motor are/should be like and were to find them
    public IntakeSubsystem() {
        this.intakeExtenderMotor = Builder.newInstance().
            port(Constants.Intake.INTAKE_EXTENDER_MOTOR_PORT).
            PID(0.1, 0.0, 0.0). // Temp Values
            outputRange(0, 360). // Temp Values
            angleLimits(-1, -1). // Temp Values
            build();
        this.intakeMotor = Builder.newInstance().
            port(Constants.Intake.MAIN_INTAKE_MOTOR_PORT).
            PID(0.1, 0.0, 0.0). // Temp Values
            outputRange(0, 360). // Temp Values
            angleLimits(-1, -1). // Temp Values
            build();
    }

    //sets the speed of the extender motor so it can be used in the extend and retract commands
    public void runExtenderMotor(double speed) {
        intakeExtenderMotor.setSpeed(speed);
    }

    //stops the extender motor so that it can be used in the extend command and retract command
    public void stopExtenderMotor() {
        intakeExtenderMotor.stopTurning();;
    }

    //stes the speed of the intake moto so it can be used in the intake and outake commands
    public void turnIntakeMotor(double speed) {
        intakeMotor.setSpeed(speed);
    }

    //stops the outtake command so that it can be use dintn intatke and outtake commands
    public void stopIntakeMotor() {
        intakeMotor.stopTurning();
    }



}
