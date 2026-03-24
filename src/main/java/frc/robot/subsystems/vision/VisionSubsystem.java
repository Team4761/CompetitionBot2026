package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class VisionSubsystem {
    private CommandSwerveDrivetrain drivetrain;
    private AprilTagFieldLayout layout;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.layout = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();
    }
}
