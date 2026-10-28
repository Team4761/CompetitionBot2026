package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class MoveArmCommand extends Command {
    
    private ArmSubsystem sub;
    private double shoulderAngleDegrees;
    private double elbowAngleDegrees;
    public MoveArmCommand(ArmSubsystem sub, double shoulderAngleDegrees, double elbowAngleDegrees) {
        this.sub = sub;
        this.shoulderAngleDegrees = shoulderAngleDegrees;
        this.elbowAngleDegrees = elbowAngleDegrees;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        sub.setShoulderAngleDegrees(shoulderAngleDegrees);
        sub.setElbowAngleDegrees(elbowAngleDegrees);
    }

    @Override
    public void end(boolean isInterrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
