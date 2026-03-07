package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DoNothingCommand;
import frc.robot.subsystems.intake.ExtendAtSpeedCommand;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class OutpostAuto extends SequentialCommandGroup{

    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public OutpostAuto(IntakeSubsystem intakeSubsystem,CommandSwerveDrivetrain drivetrain){

        final var idle = new SwerveRequest.Idle();
        
        addCommands(
        );
    }
}
