package frc.robot.basecommands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command does nothing. Its use is for the robot to stay idle.
 */
public class DoNothingCommand extends Command{

    public DoNothingCommand() {/* Magic */}
    @Override
    public boolean isFinished() { return true; }

    @Override
    public void end(boolean interrupted) {}
}
