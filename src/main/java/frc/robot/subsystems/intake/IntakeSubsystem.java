package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;

public class IntakeSubsystem extends SubsystemBase{
    // Make the code aware there should be 2 motors
    private SmartKrakenMotor intakeExtenderMotor;
    private SmartKrakenMotor intakeMotor;

    // Tell the code what those motor are/should be like and were to find them
    public IntakeSubsystem() {
        this.intakeExtenderMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Intake.INTAKE_EXTENDER_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0) // Temp Values
            .outputRange(-1.0, 1.0) // Duty cycle output limits
            .angleLimits(-1, -1) // Temp Values
            .mode(SmartKrakenMotor.MotorMode.WRAPPED)
            .build();
        this.intakeMotor = SmartKrakenMotor.Builder.newInstance()
            .port(Constants.Intake.MAIN_INTAKE_MOTOR_PORT)
            .PID(0.1, 0.0, 0.0) // Temp Values
            .outputRange(-1.0, 1.0) // Duty cycle output limits
            .angleLimits(-1, -1) // Temp Values
            .mode(SmartKrakenMotor.MotorMode.WRAPPED)
            .build();
    }

    //sets the speed of the extender motor so it can be used in the extend and retract commands
    public void runExtenderMotor(double speed) {
        
        intakeExtenderMotor.setSpeedPercent(speed);
    }

    //stops the extender motor so that it can be used in the extend command and retract command
    public void stopExtenderMotor() {
        intakeExtenderMotor.stopTurning();
    }
    /* 
    //sets an angle for the extender motor to go to
    public void setExtenderMotorAngle(double angle) {
        intakeExtenderMotor.setAngle(angle * Constants.Intake.CONVERSION_FACTOR_MtoE);
    }

    //puts the extender motor into coast mode
    public void coastExtenderMotor(){
        intakeExtenderMotor.enableCoasting();

    }

    //puts the robot back into the original mode called brake
    public void brakeExtenderMotor() {
        intakeExtenderMotor.enableBrake();
    }
    */
    //sets the speed of the intake motor so it can be used in the intake and outake commands
    public void turnIntakeMotor(double speed) {
        intakeMotor.setSpeedPercent(speed);
        //System.out.println("subsystem");
    }

    

    public void turnIntakeMotorRPM(double speed) {
        intakeMotor.setSpeed(speed);
        //System.out.println("were rotating :o");
    }

    //stops the intake motor so that it can be use in intake and outtake commands
    public void stopIntakeMotor() {
        intakeMotor.stopTurning();
    }



}
