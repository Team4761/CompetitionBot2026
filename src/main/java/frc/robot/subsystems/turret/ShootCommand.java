package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShootCommand extends Command {
    private final TurretSubsystem turretSubsystem;

    /**
     * @param sub The turret subsystem holding the shooter components
     * @param spitterSpeed Speed for the main flywheel/shooter
     * @param kickerSpeed Speed for the feed mechanism (kicker)
     */
    public ShootCommand(TurretSubsystem sub) {
        this.turretSubsystem = sub;
        
        // IMPORTANT: Intentionally NOT use addRequirements(sub) here.
        // If require the TurretSubsystem, will interrupt the TurretManualAimCommand,
        // which prevents the operator from aiming while shooting. (I think)
    }

    @Override
    public void initialize() {
        this.turretSubsystem.setSpitterMotorSpeed(Constants.Turret.SPITTER_SPEED);
        Timer.delay(Constants.Turret.KICKER_INIT_DELAY);
        this.turretSubsystem.setSpindexerMotorSpeed(Constants.Turret.SPINDEXER_SPEED); // [FIXME]: Make this a constant or passed in arg
        this.turretSubsystem.setKickerMotorSpeed(Constants.Turret.KICKER_SPEED);
    }

    @Override
    public void execute() {
        // Keep motors running
    }

    @Override
    public boolean isFinished() { 
        return false; 
    }

    @Override
    public void end(boolean isInterrupted) {
        this.turretSubsystem.stopSpitter();
        this.turretSubsystem.stopSpindexer();
        this.turretSubsystem.stopKicker();
    }
}