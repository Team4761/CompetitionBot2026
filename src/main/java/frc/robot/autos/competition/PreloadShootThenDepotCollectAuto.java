package frc.robot.autos.competition;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.autos.AutoSettings;
import frc.robot.autos.ExtendDownMoveAndGather;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveToRelativePoseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class PreloadShootThenDepotCollectAuto extends SequentialCommandGroup {
    private static final double DEPOT_APPROACH_DISTANCE_METERS =
        FieldConstants.Trench.TRENCH_DISTANCE_FROM_ALLIANCE_WALL - Constants.Robot.ROBOT_LENGTH;
    private static final double DEPOT_GATHER_DISTANCE_METERS =
        FieldConstants.Depot.DEPOT_LENGTH + Constants.Robot.ROBOT_LENGTH;

    public PreloadShootThenDepotCollectAuto(
        IntakeSubsystem intakeSubsystem,
        CommandSwerveDrivetrain drivetrain,
        TurretSubsystem turretSubsystem
    ) {
        AutoSettings settings = AutoSettings.fromDashboardSelections();

        addCommands(
            new DoNothingCommand(),
            Commands.runOnce(() -> turretSubsystem.setHorizontalMotor(0.0), turretSubsystem),
            new ShootCommand(turretSubsystem).withTimeout(4.0),
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
