package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/*
 * Generalized shoot command. It shoots in an arc.
 */
public class ShootWithPowerCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final double rpm;
    private final Timer feederDelayTimer = new Timer();
    private boolean feedersStarted;

    /**
     * @param sub The turret subsystem holding the shooter components
     * @param spitterSpeed Speed for the main flywheel/shooter
     * @param kickerSpeed Speed for the feed mechanism (kicker)
     */
    public ShootWithPowerCommand(TurretSubsystem sub, double rpm) {
        this.turretSubsystem = sub;
        this.rpm = rpm;
        
        // IMPORTANT: Intentionally NOT use addRequirements(sub) here.
        // If require the TurretSubsystem, will interrupt the TurretManualAimCommand,
        // which prevents the operator from aiming while shooting. (I think)
    }

    @Override
    public void initialize() {
        feedersStarted = false;
        feederDelayTimer.restart();
        this.turretSubsystem.setSpitterMotorSpeedRPM(this.rpm);
    }

    @Override
    public void execute() {
        // Delay feeding without stalling the scheduler thread.
        if (!feedersStarted && feederDelayTimer.hasElapsed(Constants.Turret.ShootConfig.KICKER_INIT_DELAY)) {
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
    public void end(boolean isInterrupted){
        feederDelayTimer.stop();
        this.turretSubsystem.stopSpitter();
        this.turretSubsystem.stopSpindexer();
        this.turretSubsystem.stopKicker();
    }
}
