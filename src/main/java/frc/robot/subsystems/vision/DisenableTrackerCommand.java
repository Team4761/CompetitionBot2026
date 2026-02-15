package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DisenableTrackerCommand extends InstantCommand {
    public DisenableTrackerCommand(VisionSubsystem vision) {
        super(vision::disenableTracker, vision);
    }
}
