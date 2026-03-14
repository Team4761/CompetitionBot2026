package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.basecommands.DoNothingCommand;
import frc.robot.basecommands.drivetrain.DriveToRelativePoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class OutpostAuto extends SequentialCommandGroup {
    private static final double OUTPOST_BACKUP_DISTANCE_METERS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 3.0;
    private static final double OUTPOST_WAIT_SECONDS = 6.0;
    private static final double TRACKING_PREP_ANGLE_DEGREES = -80.0;
    private static final double OUTPOST_SHOT_ANGLE_DEGREES = -Math.toDegrees(
        Math.atan2(
            FieldConstants.Field.ALLIANCE_ZONE_LENGTH / 2.0,
            FieldConstants.Field.ALLIANCE_ZONE_WIDTH / 2.0
        )
    );

    public OutpostAuto(
        TurretSubsystem turret,
        VisionSubsystem vision,
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
                -OUTPOST_BACKUP_DISTANCE_METERS,
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
