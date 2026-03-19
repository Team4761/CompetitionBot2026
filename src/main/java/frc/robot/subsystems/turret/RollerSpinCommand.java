package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;

/*
 * Spins the Roller; a "spinning basket" of sorts in the robot, feeding FUEL into the shooter.
 */
public class RollerSpinCommand extends Command{
    private double spinSpeed;
    private TurretSubsystem turretSubsystem;
    /**
     * 
     * @param sub The turret subsystem
     * @param speed The speed to spin the roller
     */
    public RollerSpinCommand(TurretSubsystem sub, double speed) {
        this.spinSpeed = speed;
        this.turretSubsystem = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        this.turretSubsystem.setRollerMotorSpeed(this.spinSpeed);
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
        turretSubsystem.stopRoller();
    }
}
