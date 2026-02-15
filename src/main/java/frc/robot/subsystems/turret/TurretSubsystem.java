package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartKrakenMotor;
import frc.robot.util.SmartKrakenMotor.Builder;

/*
 * The Turret is the part of the robot that scores FUEL. There are 4 motors:
 * - Kicker Motor: 1 Kraken x60
 * - Shooter Motor 1: Kraken x60
 * - Pitch Motor: 1 Kraken x60 (up down shooty movement)
 * - Yaw Motor: 1 Kraken x60 (side to side shooty movement)
 */
public class TurretSubsystem extends SubsystemBase {

    private SmartKrakenMotor kickerMotor; // Kicker Motor Controller
    private SmartKrakenMotor shooterMotor; // Shooter Motor Controller
    private SmartKrakenMotor pitchMotor; // Pitch Motor Controller
    private SmartKrakenMotor yawMotor; // Yaw Motor Controller

    public TurretSubsystem() {
        this.kickerMotor = Builder.newInstance().
            port(Constants.Turret.KICKER_MOTOR_PORT).
            PID(0.1, 0.0, 0.0). // Temp Values
            outputRange(0, 360). // Temp Values
            angleLimits(-1, -1). // Temp Values
            build();
        this.shooterMotor = Builder.newInstance().
            port(Constants.Turret.SHOOTER_MOTOR_PORT).
            PID(0.1, 0.0, 0.0). // Temp Values
            outputRange(0, 360). // Temp Values
            angleLimits(-1, -1). // Temp Values
            build();
        this.pitchMotor = Builder.newInstance().
            port(Constants.Turret.PITCH_MOTOR_PORT).
            PID(0.1, 0.0, 0.0). // Temp Values
            outputRange(0, 360). // Temp Values
            angleLimits(-1, -1). // Temp Values
            build();
        this.yawMotor = Builder.newInstance().
            port(Constants.Turret.YAW_MOTOR_PORT).
            PID(0.1, 0.0, 0.0). // Temp Values
            outputRange(0, 360). // Temp Values
            angleLimits(-1, -1). // Temp Values
            build();

    }

}