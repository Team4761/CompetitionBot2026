package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/*
 * Generalized shoot command. It shoots in an arc.
 */
public class ShootCommandSTUTTER extends Command {
    private final TurretSubsystem turretSubsystem;
    private final Timer feederDelayTimer = new Timer();
    private boolean feedersStarted;

    /**
     * @param sub The turret subsystem holding the shooter components
     * @param spitterSpeed Speed for the main flywheel/shooter
     * @param kickerSpeed Speed for the feed mechanism (kicker)
     */
    public ShootCommandSTUTTER(TurretSubsystem sub) {
        this.turretSubsystem = sub;
        
        // IMPORTANT: Intentionally NOT use addRequirements(sub) here.
        // If require the TurretSubsystem, will interrupt the TurretManualAimCommand,
        // which prevents the operator from aiming while shooting. (I think)
    }

    @Override
    public void initialize() {
        feedersStarted = false;
        feederDelayTimer.restart();
        this.turretSubsystem.spitterMotor.setRawSpeed(Constants.Turret.ShootConfig.SPITTER_SPEED);
    }

    @Override
    public void execute() {
        // Delay feeding without stalling the scheduler thread.
        if (!feedersStarted && feederDelayTimer.hasElapsed(Constants.Turret.ShootConfig.KICKER_INIT_DELAY)) {
            this.turretSubsystem.spindexerMotor.setRawSpeedPercent(Constants.Turret.ShootConfig.SPINDEXER_SPEED);
            this.turretSubsystem.kickerMotor.setRawSpeedPercent(Constants.Turret.ShootConfig.KICKER_SPEED);
            feedersStarted = true;
        }

        double onTime = 0.2; // Time to run spindexer before pausing to allow spinup of flywheel
        double offTime = 0.1; // Time to pause spindexer to allow flywheel to catch up
        double cycleTime = onTime + offTime;
        double currentTime = feederDelayTimer.get() % cycleTime;

        if (currentTime < onTime) {
            // ON stage
            // Set speed back to spindexer
            this.turretSubsystem.spindexerMotor.setRawSpeedPercent(Constants.Turret.ShootConfig.SPINDEXER_SPEED);
        } else {
            // OFF stage
            // Pause spindexer to allow flywheel to catch up
            this.turretSubsystem.spindexerMotor.stopTurning();
        }
    }

    @Override
    public boolean isFinished() { 
        return false; 
    }

    @Override
    public void end(boolean isInterrupted){
        feederDelayTimer.stop();
        this.turretSubsystem.spitterMotor.stopTurning();
        this.turretSubsystem.spindexerMotor.stopTurning();
        this.turretSubsystem.kickerMotor.stopTurning();
    }
}
