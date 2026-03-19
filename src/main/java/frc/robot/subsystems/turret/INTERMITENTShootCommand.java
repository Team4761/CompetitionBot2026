package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/*
 * Generalized shoot command. It shoots in an arc.
 */
public class INTERMITENTShootCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final Timer feederDelayTimer = new Timer();
    private boolean feedersStarted;
    private boolean didResetThisCycle;
    private boolean didReverseThisCycle;
    private Double rpm; 

    /**
     * @param sub The turret subsystem holding the shooter components
     * @param spitterSpeed Speed for the main flywheel/shooter
     * @param kickerSpeed Speed for the feed mechanism (kicker)
     */
    public INTERMITENTShootCommand(TurretSubsystem sub, Double rpm) {
        this.turretSubsystem = sub;
        this.rpm = rpm;
        
        // IMPORTANT: Intentionally NOT use addRequirements(sub) here.
        // If require the TurretSubsystem, will interrupt the TurretManualAimCommand,
        // which prevents the operator from aiming while shooting. (I think)
    }

    @Override
    public void initialize() {
        feedersStarted = false;
        didResetThisCycle = false;
        didReverseThisCycle = false;
        feederDelayTimer.restart();
        this.turretSubsystem.setSpitterMotorSpeedRPM(this.rpm);
    }

    @Override
    public void execute() {
        // Delay feeding without stalling the scheduler thread.
        if (!feedersStarted && feederDelayTimer.hasElapsed(Constants.Turret.ShootConfig.KICKER_INIT_DELAY)) {
            this.turretSubsystem.setRollerMotorSpeed(Constants.Turret.ShootConfig.ROLLER_SPEED);
            this.turretSubsystem.setKickerMotorSpeed(Constants.Turret.ShootConfig.KICKER_SPEED);
            feedersStarted = true;
        }

        if (feederDelayTimer.get() % 5.0 < 0.3 && !this.didReverseThisCycle) {
            this.didResetThisCycle = false;
            this.didReverseThisCycle = true;
            this.turretSubsystem.setRollerMotorSpeed(-1 * Constants.Turret.ShootConfig.ROLLER_SPEED);
            this.turretSubsystem.setKickerMotorSpeed(-1 * Constants.Turret.ShootConfig.KICKER_SPEED);
        } else if (feederDelayTimer.get() % 5.0 >= 0.4 && !this.didResetThisCycle) {
            this.didResetThisCycle = true;
            this.didReverseThisCycle = false;
            this.turretSubsystem.setRollerMotorSpeed(Constants.Turret.ShootConfig.ROLLER_SPEED);
            this.turretSubsystem.setKickerMotorSpeed(Constants.Turret.ShootConfig.KICKER_SPEED);
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
        this.turretSubsystem.stopRoller();
        this.turretSubsystem.stopKicker();
    }
}
