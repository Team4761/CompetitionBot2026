package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class MoveArmCommand extends Command {
    
    private ArmSubsystem sub;
    private double targetShoulderAngleDegrees;
    private double targetElbowAngleDegrees;

    public MoveArmCommand(ArmSubsystem sub, double targetShoulderAngleDegrees, double targetElbowAngleDegrees) {
        this.sub = sub;
        this.targetShoulderAngleDegrees = targetShoulderAngleDegrees;
        this.targetElbowAngleDegrees = targetElbowAngleDegrees;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        sub.setShoulderAngleDegrees(this.targetShoulderAngleDegrees);
        sub.setElbowAngleDegrees(this.targetElbowAngleDegrees);
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
