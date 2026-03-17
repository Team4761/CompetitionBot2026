package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.basecommands.DoNothingCommand;
import frc.robot.basecommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.basecommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ShootWithPowerCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class MiddleBackupAuto extends SequentialCommandGroup {
    public MiddleBackupAuto(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret, IntakeSubsystem intake) {
        addCommands(
            new DoNothingCommand(),
            drivetrain.runOnce(drivetrain::seedFieldCentric), // [FIXME] This is an error. It field cetric's backwards
            new DriveRelativeMetersCommand(drivetrain, 0.5, 0.0),
            new ExtendCommand(intake),
            new ShootWithPowerCommand(turret, Constants.Turret.ShootConfig.AUTO_SPITTER_SPEED).withTimeout(4)
        );
    }
}
