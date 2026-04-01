package frc.robot.subsystems.swerve.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class ADJUSTEDSnapToHubCommand extends Command {

    private CommandSwerveDrivetrain drive;
    /**
     * 
     * @param drivetrain //drivetrain
     */
    public ADJUSTEDSnapToHubCommand(CommandSwerveDrivetrain drivetrain) {
        this.drive = drivetrain;
        
        addRequirements(drive);
    }

    public Command getCommand() {
        return Commands.defer(() -> {
            Pose2d currentPos = drive.getState().Pose;
            double currentDirection = Math.toDegrees(drive.getState().Pose.getRotation().getRadians());

            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            double hubX = alliance == DriverStation.Alliance.Blue ? FieldConstants.Hub.HUB_BLUE_X : FieldConstants.Hub.HUB_RED_X;
            double hubY = alliance == DriverStation.Alliance.Blue ? FieldConstants.Hub.HUB_BLUE_Y : FieldConstants.Hub.HUB_RED_Y;

            Transform2d toHub = new Transform2d(currentPos, new Pose2d(hubX, hubY, currentPos.getRotation()));
            double angleToHub = Math.toDegrees(toHub.getRotation().getRadians());
            angleToHub = Math.atan((currentPos.getY() - hubY)/(currentPos.getX() - hubX));
            
            return new DriveCommand(drive, 0, 0, angleToHub - currentDirection);
        }, Set.of(drive));
    }
    
}
