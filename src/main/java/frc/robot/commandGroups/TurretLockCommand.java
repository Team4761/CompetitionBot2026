package frc.robot.commandGroups;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemBAD;
import frc.robot.subsystems.vision.VisionSubsystemBAD.TurretAimOutput;

public class TurretLockCommand extends Command {
    public static final String ENABLED_DASHBOARD_KEY = "Turret Lock Enabled";

    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystemBAD visionSubsystem;
    private boolean wasEnabled;

    public TurretLockCommand(
        TurretSubsystem turretSubsystem,
        VisionSubsystemBAD visionSubsystem
    ) {
        this.turretSubsystem = turretSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        wasEnabled = false;
        visionSubsystem.resetTurretTrackingState();
    }

    @Override
    public void execute() {
        boolean isEnabled = SmartDashboard.getBoolean(ENABLED_DASHBOARD_KEY, false);
        if (!isEnabled) {
            if (wasEnabled) {
                visionSubsystem.resetTurretTrackingState();
            }
            wasEnabled = false;
            turretSubsystem.stopHorizontal();
            return;
        }

        if (!wasEnabled) {
            visionSubsystem.resetTurretTrackingState();
        }
        wasEnabled = true;

        Optional<TurretAimOutput> aimOutput = visionSubsystem.getTurretAimOutput();
        if (aimOutput.isEmpty()) {
            turretSubsystem.setHorizontalMotor(0.0);
            return;
        }

        TurretAimOutput output = aimOutput.get();
        turretSubsystem.turnHorizontalMotor(output.horizontalTurnDegrees());
        turretSubsystem.setVerticalMotor(output.verticalLaunchAngleDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        wasEnabled = false;
        visionSubsystem.resetTurretTrackingState();
        turretSubsystem.stopHorizontal();
    }
}
