package frc.robot.autos.competition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.autos.AutoSettings;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.baseCommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class PreloadShootThenTrenchCollectAuto extends SequentialCommandGroup {
    private static final double FORWARD_TO_TRENCH_ENTRY_METERS =
        FieldConstants.Trench.TRENCH_DISTANCE_FROM_ALLIANCE_WALL - Constants.Robot.ROBOT_LENGTH;
    private static final double TRENCH_SWEEP_METERS = FieldConstants.Trench.TRENCH_LENGTH;

    public PreloadShootThenTrenchCollectAuto(
        IntakeSubsystem intakeSubsystem,
        CommandSwerveDrivetrain drivetrain,
        TurretSubsystem turretSubsystem
    ) {
        AutoSettings settings = AutoSettings.fromDashboardSelections();
        double shiftToCenterMeters = settings.shiftMetersTo(AutoSettings.StartingPosition.CENTER);
        double trenchEntryTurnDegrees = Math.toDegrees(
            Math.atan2(shiftToCenterMeters, FORWARD_TO_TRENCH_ENTRY_METERS)
        );
        double trenchApproachDistanceMeters =
            Math.hypot(FORWARD_TO_TRENCH_ENTRY_METERS, shiftToCenterMeters);

        addCommands(
            new DoNothingCommand(),
            /*check if
                red team ->
                if sees april tag 10 rotated at -45 degrees around the vertical axis(applies to all other angles) and/or 11 at 45
                or
                if sees april tag 9 at 45 and 8 at -45

                blue team ->
                if sees april tag 26 at -45 degrees and/or 27 at 45
                or
                if sees april tag 25 at 45 and 24 at -45

                make corrections so it sees those tags at those angles
            */
            Commands.runOnce(() -> turretSubsystem.setHorizontalMotor(0.0), turretSubsystem),
            new ShootCommand(turretSubsystem).withTimeout(4.0),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new ExtendCommand(intakeSubsystem),
            Commands.deadline(
                Commands.sequence(
                    buildTurnIfNeeded(drivetrain, trenchEntryTurnDegrees),
                    /* check if
                        previos april tag was 10 or 11
                            sees april tag 12 at at 0 degrees around the vertical axis(applies to all other angles)
                        previos tag was 8 or 9
                            sees april tag 7 at 0
                        previos tag was 26 or 27
                            sees april tag 28 at 0
                        previos tag was 25 or 24
                            sees tag 23 at 0

                            make corrections if the april tag is visible but not a 0
                    */
                    new DriveRelativeMetersCommand(drivetrain, trenchApproachDistanceMeters, 0.0),
                    buildTurnIfNeeded(drivetrain, -trenchEntryTurnDegrees),
                    new DriveRelativeMetersCommand(drivetrain, TRENCH_SWEEP_METERS, 0.0)
                ),
                new IntakeCommand(intakeSubsystem)
            ),
            new JostleCommand(intakeSubsystem).withTimeout(1.0)
        );
    }

    private static Command buildTurnIfNeeded(
        CommandSwerveDrivetrain drivetrain,
        double deltaDegrees
    ) {
        return Math.abs(deltaDegrees) < 1e-6
            ? Commands.none()
            : new RotateRelativeDegreesCommand(drivetrain, deltaDegrees);
    }
}
