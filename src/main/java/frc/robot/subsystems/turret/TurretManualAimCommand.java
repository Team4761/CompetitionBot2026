package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class TurretManualAimCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final DoubleSupplier aoaSupplier;  // Adjusts AOA (Vertical)
    private final DoubleSupplier turnSupplier; // Turns the turret (Horizontal)

    /**
     * @param sub The turret subsystem
     * @param aoaSupplier Left joystick Y-axis supplier for vertical control (AOA)
     * @param turnSupplier Right joystick X-axis supplier for horizontal control (Turn)
     */
    public TurretManualAimCommand(TurretSubsystem sub, DoubleSupplier aoaSupplier, DoubleSupplier turnSupplier) {
        this.turretSubsystem = sub;
        this.aoaSupplier = aoaSupplier;
        this.turnSupplier = turnSupplier;
        
        // Requires the turret so it can act as the default command
        addRequirements(sub);
    }

    @Override
    public void execute() {
        // Read joystick values and set motor speeds directly
        turretSubsystem.setVerticalMotor(aoaSupplier.getAsDouble());
        turretSubsystem.setHorizontalMotor(turnSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() { 
        // Default drive/aim command so it never finishes on its own
        return false; 
    }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.setVerticalMotor(0);
        turretSubsystem.setHorizontalMotor(0);
    }
}