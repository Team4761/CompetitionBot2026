package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command spins the kicker.
 */
public class KickerSpinCommand extends Command{
    private double spinSpeed;
    private TurretSubsystem turretSubsystem;
    /**
     * 
     * @param sub The turret subsystem
     * @param speed The speed to run the kicker
     */
    public KickerSpinCommand(TurretSubsystem sub, double speed) {
        this.spinSpeed = speed;
        this.turretSubsystem = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        this.turretSubsystem.setKickerMotorSpeed(this.spinSpeed);
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
        turretSubsystem.stopKicker();
    }
}
