package frc.robot.commandGroups;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TurretAimOutput;

public class TurretLockCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;

    public TurretLockCommand(
        TurretSubsystem turretSubsystem,
        VisionSubsystem visionSubsystem
    ) {
        this.turretSubsystem = turretSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.resetTurretTrackingState();
    }

    @Override
    public void execute() {
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
        visionSubsystem.resetTurretTrackingState();
        turretSubsystem.stopHorizontal();
    }
}
