package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.basecommands.DoNothingCommand;
import frc.robot.basecommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.basecommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.ShootWithPowerCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class MiddleBackupAuto extends SequentialCommandGroup {
    private static final double AUTO_DISTANCE_METERS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.25;

    public MiddleBackupAuto(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret, IntakeSubsystem intake) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric), // [FIXME] This is an error. It field cetric's backwards
            new DriveRelativeMetersCommand(drivetrain, -AUTO_DISTANCE_METERS, 0.0),
            new ExtendCommand(intake),
            new ShootWithPowerCommand(turret, Constants.Turret.ShootConfig.AUTO_SPITTER_SPEED).withTimeout(4)
        );
    }
}
