package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This command is a command for the Elastic Dashboard: it retrieves the horizontal & vertical encoder values for the turret
*/
public class GetPoseCommand extends InstantCommand{
    public GetPoseCommand(CommandSwerveDrivetrain sub) {
        super(() -> {sub.registerTelemetry((state) ->{
            SmartDashboard.putNumber("Pose X", state.Pose.getX());
            SmartDashboard.putNumber("Pose Y", state.Pose.getY());
            SmartDashboard.putNumber("Pose Rotation", state.Pose.getRotation().getDegrees());
        });});
    }
}

