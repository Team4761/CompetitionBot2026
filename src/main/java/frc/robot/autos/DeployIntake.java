package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.basecommands.DoNothingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ShootAtAngleCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretAimChangeCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.ShootWithPowerCommand;

public class DeployIntake extends SequentialCommandGroup{

    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public DeployIntake(IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem){

        final var idle = new SwerveRequest.Idle();
         
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new ExtendCommand(intakeSubsystem),
            //new TurretAimChangeCommand(turretSubsystem, () -> 0, () -> 15),
            new ShootWithPowerCommand(turretSubsystem, Constants.Turret.ShootConfig.AUTO_SPITTER_SPEED).withTimeout(4)
        );
        
    }
}