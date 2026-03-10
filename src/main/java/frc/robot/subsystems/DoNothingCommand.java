package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command does nothing. Its use is for the robot to stay idle.
 */
public class DoNothingCommand extends Command{

    public DoNothingCommand() {/* Magic */}
    public boolean isFinished() { return true; }
    public void end() {}
}
