package frc.robot.autos.testing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.ExtendDownMoveAndGather;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeGatherTestAuto extends SequentialCommandGroup {
    public IntakeGatherTestAuto(IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            new ExtendDownMoveAndGather(intakeSubsystem, drivetrain)
        );
    }
}
