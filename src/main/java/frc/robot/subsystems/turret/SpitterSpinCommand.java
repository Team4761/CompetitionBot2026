package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Spins the Spitter; the last wheel before the FUEL is ejected out of the shooter.
 */
public class SpitterSpinCommand extends Command{
    private double spinSpeed;
    private TurretSubsystem turretSubsystem;

    /**
     * 
     * @param sub The turret subsystem
     * @param speed The speed to run the spitter
     */
    public SpitterSpinCommand(TurretSubsystem sub, double speed) {
        this.spinSpeed = speed;
        this.turretSubsystem = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        this.turretSubsystem.setSpitterMotorSpeed(spinSpeed);
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
        turretSubsystem.stopSpitter();
    }
}
