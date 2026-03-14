package frc.robot.autos.testing;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.autos.AutoSettings;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveToRelativePoseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class OutpostLaneShotTestAuto extends SequentialCommandGroup {
    private static final double OUTPOST_APPROACH_DISTANCE_METERS =
        FieldConstants.Trench.TRENCH_DISTANCE_FROM_ALLIANCE_WALL - Constants.Robot.ROBOT_LENGTH;
    private static final double OUTPOST_WAIT_SECONDS = 6.0;
    private static final double TRACKING_PREP_ANGLE_DEGREES = -80.0;
    private static final double OUTPOST_SHOT_ANGLE_DEGREES = -Math.toDegrees(
        Math.atan2(
            FieldConstants.Field.ALLIANCE_ZONE_LENGTH / 2.0,
            FieldConstants.Field.ALLIANCE_ZONE_WIDTH / 2.0
        )
    );

    public OutpostLaneShotTestAuto(
        TurretSubsystem turret,
        CommandSwerveDrivetrain drivetrain
    ) {
        AutoSettings settings = AutoSettings.fromDashboardSelections();

        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            // this set command could use side hub AprilTags
            Commands.runOnce(() -> turret.setHorizontalMotor(TRACKING_PREP_ANGLE_DEGREES), turret),
            new DriveToRelativePoseCommand(
                drivetrain,
                OUTPOST_APPROACH_DISTANCE_METERS,
                settings.shiftMetersTo(AutoSettings.StartingPosition.RIGHT),
                0.0
            ),
            Commands.waitSeconds(OUTPOST_WAIT_SECONDS),
            // currently it could see the april tags on the front side of the hub
            Commands.runOnce(() -> turret.setHorizontalMotor(OUTPOST_SHOT_ANGLE_DEGREES), turret),
            new ShootCommand(turret).withTimeout(6.0)
        );
    }
}
