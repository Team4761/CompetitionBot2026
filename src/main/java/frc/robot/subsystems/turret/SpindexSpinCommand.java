package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;

public class SpindexSpinCommand extends Command{
    private double spinSpeed;
    private TurretSubsystem turretSubsystem;

    public SpindexSpinCommand(TurretSubsystem sub, double speed) {
        this.spinSpeed = speed;
        this.turretSubsystem = sub;
    }

    @Override
    public void execute() {
        this.turretSubsystem.setSpindexerMotorSpeed(this.spinSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.stopSpindexer();
    }
}