package frc.robot.subsystems.turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ElasticManualOverrideCommand extends Command{
    private TurretSubsystem turretSubsystem;
    private BooleanSupplier override;
    public ElasticManualOverrideCommand(TurretSubsystem sub, BooleanSupplier override) {
        this.turretSubsystem = sub;
        this.override = override;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Manual Turret Control", override.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
    }
}
