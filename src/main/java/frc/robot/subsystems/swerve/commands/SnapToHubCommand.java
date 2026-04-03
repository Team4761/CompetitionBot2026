package frc.robot.subsystems.swerve.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SnapToHubCommand extends Command {

    private CommandSwerveDrivetrain drive;
    /**
     * 
     * @param drivetrain
     */
    public SnapToHubCommand(CommandSwerveDrivetrain drivetrain) {
        this.drive = drivetrain;
        
        addRequirements(drive);
    }

    public Command getCommand() {
        return Commands.defer(() -> {
            Pose2d currentPos = drive.getState().Pose;

            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            double hubX = alliance == DriverStation.Alliance.Blue ? FieldConstants.Hub.HUB_BLUE_X : FieldConstants.Hub.HUB_RED_X;
            double hubY = alliance == DriverStation.Alliance.Blue ? FieldConstants.Hub.HUB_BLUE_Y : FieldConstants.Hub.HUB_RED_Y;

            double angleToHub = Math.toDegrees(Math.atan2(hubY - currentPos.getY(), hubX - currentPos.getX()));
            
            return new DriveCommand(drive, 0, 0, angleToHub);
        }, Set.of(drive));
    }
}
