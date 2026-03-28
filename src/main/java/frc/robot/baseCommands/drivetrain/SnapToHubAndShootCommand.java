// package frc.robot.baseCommands.drivetrain;

// import java.util.Set;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.FieldConstants;
// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import frc.robot.subsystems.turret.CalculatedHubShotCommand;
// import frc.robot.subsystems.turret.TurretSubsystem;

// public class SnapToHubAndShootCommand extends Command {

//     private CommandSwerveDrivetrain drive;
//     private TurretSubsystem turret;
//     private Pose2d currentPos;
//     private double currentDirection;
//     private double hubX;
//     private double hubY;
//     private double angleToHub;
//     /**
//      * 
//      * @param drivetrain //drivetrain
//      */
//     public SnapToHubAndShootCommand(CommandSwerveDrivetrain drivetrain, TurretSubsystem turretSubsystem) {
//         this.drive = drivetrain;
//         this.turret = turretSubsystem;
        
//         addRequirements(drive);
//     }

//     public Command getCommand() {
//         return Commands.defer(() -> {
//             this.currentPos = drive.getState().Pose;
//             this.currentDirection = Math.toDegrees(drive.getState().Pose.getRotation().getRadians());
//             this.hubX = ("BLUE".equals(Constants.Field.ALLIANCE_COLOR) ? FieldConstants.Hub.HUB_BLUE_X : FieldConstants.Hub.HUB_RED_X);
//             this.hubY = ("BLUE".equals(Constants.Field.ALLIANCE_COLOR) ? FieldConstants.Hub.HUB_BLUE_Y : FieldConstants.Hub.HUB_RED_Y);
//             this.angleToHub = Math.atan((this.currentPos.getY() - this.hubY)/(this.currentPos.getX() - this.hubX));
//             if (this.hubY > this.currentPos.getY()) {
//                 if (this.angleToHub > 0) {
//                     this.angleToHub = (-1 * 180) + this.angleToHub;
//                 }
//             }
//             else {
//                 if (this.angleToHub < 0) {
//                     this.angleToHub = 180 + this.angleToHub;
//                 }
//             }
//             if (this.angleToHub == 0) {
//                 if (this.currentPos.getX() > this.hubX) {
//                     this.angleToHub = 180;
//                 }
//             }
//             return new SequentialCommandGroup(new DriveCommand(drive, 0, 0, angleToHub - currentDirection), new CalculatedHubShotCommand(turret, drive));
//         }, Set.of(drive,turret));
//     }
    
// }
