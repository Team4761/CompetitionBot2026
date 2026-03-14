package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.coreCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;

public class ExtendDownMoveAndGather extends SequentialCommandGroup {
    private static final double DEFAULT_GATHER_DISTANCE_METERS =
        FieldConstants.Depot.DEPOT_LENGTH + Constants.Robot.ROBOT_LENGTH;

    public ExtendDownMoveAndGather(IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain drivetrain) {
        this(intakeSubsystem, drivetrain, DEFAULT_GATHER_DISTANCE_METERS);
    }

    public ExtendDownMoveAndGather(
        IntakeSubsystem intakeSubsystem,
        CommandSwerveDrivetrain drivetrain,
        double gatherDistanceMeters
    ) {
        addCommands(
            new DoNothingCommand(),
            new ExtendCommand(intakeSubsystem),
            Commands.deadline(
                new DriveRelativeMetersCommand(drivetrain, gatherDistanceMeters, 0.0),
                new IntakeCommand(intakeSubsystem)
            ),
            new JostleCommand(intakeSubsystem).withTimeout(1.0)
        );
    }
}
