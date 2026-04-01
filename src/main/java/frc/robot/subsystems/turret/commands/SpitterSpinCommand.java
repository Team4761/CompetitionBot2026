package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Spins the Spitter; the last wheel before the FUEL is ejected out of the shooter.
 */
public class SpitterSpinCommand extends Command{
    private double spinRPM;
    private TurretSubsystem turretSubsystem;

    /**
     * 
     * @param sub The turret subsystem
     * @param RPM The RPM to run the spitter
     */
    public SpitterSpinCommand(TurretSubsystem sub, double RPM) {
        this.spinRPM = RPM;
        this.turretSubsystem = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        this.turretSubsystem.spitterMotor.setRawSpeed(this.spinRPM);
    }

    @Override
    public void execute() {
        // Keep command instance open to allow it to cleanly end
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.spitterMotor.stopTurning();
    }
}
