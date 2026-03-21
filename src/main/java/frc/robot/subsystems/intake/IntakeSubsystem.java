package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;
import frc.robot.subsystems.leds.LEDSubsystem;

public class IntakeSubsystem extends SubsystemBase{
    // Make the code aware there should be 2 motors
    private SmartKrakenMotor intakeExtenderMotor;
    private SmartKrakenMotor intakeMotor;

    // Tell the code what those motor are/should be like and were to find them
    public IntakeSubsystem() {
        this.intakeExtenderMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Intake.INTAKE_EXTENDER_MOTOR_PORT)
            .PID(0.5, 0.0, 0.0) // Temp Values
            .outputRange(-1.0, 1.0) // Duty cycle output limits
            .angleLimits(Constants.Intake.MIN_EXTENSION_ANGLE * Constants.Intake.CONVERSION_FACTOR_MtoE, 
                    Constants.Intake.MAX_EXTENSION_ANGLE * Constants.Intake.CONVERSION_FACTOR_MtoE)
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .gearRatio(Constants.Intake.CONVERSION_FACTOR_MtoE)
            .build();
        this.intakeMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Intake.MAIN_INTAKE_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0) // Temp Values
            .outputRange(-1.0, 1.0) // Duty cycle output limits
            .angleLimits(-1, -1) // Temp Values
            .mode(SmartKrakenMotor.MotorMode.CONTINUOUS)
            .build();
    }

    // sets the speed of the extender motor so it can be used in the extend and retract commands
    public void runExtenderMotor(double speed) {
        System.out.println("running extender motor");
        intakeExtenderMotor.setRawSpeedPercent(speed);
    }

    // stops the extender motor so that it can be used in the extend command and retract command
    public void stopExtenderMotor() {
        intakeExtenderMotor.stopTurning();
    }
    
    //sets an angle for the extender motor to go to
    public void setExtenderMotorAngle(double angle) {
        intakeExtenderMotor.set(angle * Constants.Intake.CONVERSION_FACTOR_MtoE);
    }
    public void turnExtenderMotorAngle(double angle) {
        intakeExtenderMotor.turn(angle * Constants.Intake.CONVERSION_FACTOR_MtoE);
    }
    public double getExtenderMotorAngle() {
        return intakeExtenderMotor.getAngle() * Constants.Intake.CONVERSION_FACTOR_EtoM;
    }
    public void enableExtenderCoasting(){
        intakeExtenderMotor.enableCoasting();
    }

    public void disableExtenderCoasting() {
        intakeExtenderMotor.disableCoasting();
    }

    public void turnIntakeMotor(double speed) {
        intakeMotor.setRawSpeedPercent(speed);
    }

    public void turnIntakeMotorRPM(double speed) {
        intakeMotor.setRawSpeed(speed);
    }

    //stops the intake motor so that it can be use in intake and outtake commands
    public void stopIntakeMotor() {
        intakeMotor.stopTurning();
    }

    public void enableIntakeCoasting() {
        intakeMotor.enableCoasting();
    }

    public void disableIntakeCoasting() {
        intakeMotor.disableCoasting();
    }

    public static Color intakeToElastic() {
        Color color = null;
        if (LEDSubsystem.intakeTrue == 1.0) {
            color = new Color(255, 50, 50);
        }
        else{
            color = new Color(50, 255, 50);
        }
        return color;
    }
}
