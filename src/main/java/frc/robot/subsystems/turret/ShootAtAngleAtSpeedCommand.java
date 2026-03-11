package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/**
 * This command is utilized more for autonomous. Controls the hood and shooter. Sets a firing angle and shoots.
 */
public class ShootAtAngleAtSpeedCommand extends Command{

    private final TurretSubsystem turretSubsystem;
    private final double shootSpeed;
    private final double shootAngle;

    /**
     * @param sub The turret subsystem holding the shooter components
     * @param speed % of max speed (0-1)
     * @param angle launch angle in degrees above horizontal
     */
    public ShootAtAngleAtSpeedCommand(TurretSubsystem sub, double angle, double speed) {
        this.turretSubsystem = sub;
        this.shootSpeed = speed;
        this.shootAngle = angle;
        
        // IMPORTANT: Intentionally NOT use addRequirements(sub) here.
        // If require the TurretSubsystem, will interrupt the TurretManualAimCommand,
        // which prevents the operator from aiming while shooting. (I think)
    }

    @Override
    public void initialize() {
        this.turretSubsystem.setSpitterMotorSpeed(shootSpeed * Constants.Turret.ShootConfig.SPITTER_SPEED);
        Timer.delay(Constants.Turret.ShootConfig.KICKER_INIT_DELAY);
        this.turretSubsystem.setSpindexerMotorSpeed(Constants.Turret.ShootConfig.SPINDEXER_SPEED * shootSpeed); 
        this.turretSubsystem.setKickerMotorSpeed(Constants.Turret.ShootConfig.KICKER_SPEED * shootSpeed);

        this.turretSubsystem.setVerticalMotor(shootAngle);
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
