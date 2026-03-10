package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShootCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private boolean feedersStarted;
    private final Timer shotTimer = new Timer();

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
        feedersStarted = false;
        shotTimer.restart();
        this.turretSubsystem.setSpitterMotorSpeed(Constants.Turret.ShootConfig.SPITTER_SPEED);
    }

    @Override
    public void execute() {
        if (!feedersStarted && shotTimer.hasElapsed(Constants.Turret.ShootConfig.KICKER_INIT_DELAY)) {
            this.turretSubsystem.setSpindexerMotorSpeed(Constants.Turret.ShootConfig.SPINDEXER_SPEED);
            this.turretSubsystem.setKickerMotorSpeed(Constants.Turret.ShootConfig.KICKER_SPEED);
            feedersStarted = true;
        }
    }

    @Override
    public boolean isFinished() { 
        return false; 
    }

    @Override
    public void end(boolean isInterrupted) {
        shotTimer.stop();
        this.turretSubsystem.stopSpitter();
        this.turretSubsystem.stopSpindexer();
        this.turretSubsystem.stopKicker();
    }
}
