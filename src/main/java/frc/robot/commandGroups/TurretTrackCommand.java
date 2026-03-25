package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretTrackCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final CommandSwerveDrivetrain drivetrainSubsystem;

    public TurretTrackCommand(
        TurretSubsystem turretSubsystem,
        CommandSwerveDrivetrain drivetrainSubsystem
    ) {
        this.turretSubsystem = turretSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        // Intentionally no requirement so this can keep running alongside other turret commands.
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (RobotContainer.isVisionTrackingEnabled()) {
            turretSubsystem.stepHorizontalMotor(drivetrainSubsystem.getHubAimErrorDegrees());
        }
        else {
            turretSubsystem.stopHorizontal();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.stopHorizontal();
    }
}
