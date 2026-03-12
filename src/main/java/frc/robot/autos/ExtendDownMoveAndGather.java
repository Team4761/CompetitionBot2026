package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;

public class ExtendDownMoveAndGather extends SequentialCommandGroup{

    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ExtendDownMoveAndGather(IntakeSubsystem intakeSubsystem,CommandSwerveDrivetrain drivetrain){

        final var idle = new SwerveRequest.Idle();
         
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new ExtendCommand(intakeSubsystem),
            new IntakeCommand(intakeSubsystem),
            drivetrain.runOnce(drivetrain::seedFieldCentric),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(AUTO_SPEED_MPS)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(10.0),
            drivetrain.applyRequest(() -> idle).withTimeout(0.1),
            new JostleCommand(intakeSubsystem).withTimeout(1)

        );
        
    }
}
