package frc.robot.subsystems.arm.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class MoveArmCommand extends Command {
    
    private ArmSubsystem sub;
    private DoubleSupplier targetShoulderAngleDegrees;
    private DoubleSupplier targetElbowAngleDegrees;

    public MoveArmCommand(ArmSubsystem sub, DoubleSupplier targetShoulderAngleDegrees, DoubleSupplier targetElbowAngleDegrees) {
        this.sub = sub;
        this.targetShoulderAngleDegrees = targetShoulderAngleDegrees;
        this.targetElbowAngleDegrees = targetElbowAngleDegrees;
        addRequirements(sub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //System.out.println(sub.elbowMotor.getAngle());
        sub.setShoulderAngleDegrees(this.targetShoulderAngleDegrees.getAsDouble()*Constants.Arm.STICK_TO_DEGREES);
        sub.setElbowAngleDegrees(this.targetElbowAngleDegrees.getAsDouble()*Constants.Arm.STICK_TO_DEGREES);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        sub.elbowMotor.stopTurning();
        sub.shoulderMotor.stopTurning();
    }
}
