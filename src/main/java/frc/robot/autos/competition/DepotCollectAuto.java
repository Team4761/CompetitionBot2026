package frc.robot.autos.competition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.autos.AutoSettings;
import frc.robot.autos.ExtendDownMoveAndGather;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveToRelativePoseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DepotCollectAuto extends SequentialCommandGroup {
    private static final double DEPOT_APPROACH_DISTANCE_METERS =
        FieldConstants.Trench.TRENCH_DISTANCE_FROM_ALLIANCE_WALL - Constants.Robot.ROBOT_LENGTH;
    private static final double DEPOT_GATHER_DISTANCE_METERS =
        FieldConstants.Depot.DEPOT_LENGTH + Constants.Robot.ROBOT_LENGTH;

    public DepotCollectAuto(IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain drivetrain) {
        AutoSettings settings = AutoSettings.fromDashboardSelections();

        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new DriveToRelativePoseCommand(
                drivetrain,
                DEPOT_APPROACH_DISTANCE_METERS,
                settings.shiftMetersTo(AutoSettings.StartingPosition.LEFT),
                0.0
            ),
            new ExtendDownMoveAndGather(intakeSubsystem, drivetrain, DEPOT_GATHER_DISTANCE_METERS)
        );
    }
}
