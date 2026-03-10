package frc.robot.subsystems.turret;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElasticManualOverrideCommand extends InstantCommand{
    public ElasticManualOverrideCommand(BooleanSupplier override) {
        super(() -> SmartDashboard.putBoolean("Manual Turret Control", override.getAsBoolean()));
    }
}
