package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.baseCommands.drivetrain.DriveCommand;
import frc.robot.baseCommands.drivetrain.DriveRelativeMetersCommand;
import frc.robot.baseCommands.drivetrain.DriveToRelativePoseCommand;
import frc.robot.baseCommands.drivetrain.RotateRelativeDegreesCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.ExtendCommand;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;
import frc.robot.subsystems.intake.STUTTERExtendCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ShootAtAngleCommand;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretAimChangeCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.ShootWithPowerCommand;

public class NeutralZoneAuto extends SequentialCommandGroup{

    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public NeutralZoneAuto(IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, CommandSwerveDrivetrain drivetrain){

        final var idle = new SwerveRequest.Idle();
        final double neutralTraversalDistance = FieldConstants.Trench.TRENCH_WIDTH + FieldConstants.Field.NEUTRAL_ZONE_LENGTH/3.0;
         
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            new ExtendCommand(intakeSubsystem),//replace with better extend command[TODO]
            //new TurretAimChangeCommand(turretSubsystem, () -> 0, () -> 15),
            new ShootWithPowerCommand(turretSubsystem, Constants.Turret.ShootConfig.AUTO_SPITTER_SPEED).withTimeout(4),
            //new RotateRelativeDegreesCommand(drivetrain, "LEFT".equals(Constants.Field.STARTING_POSITION) ? -60.0 : 60.0),
            new DriveCommand(drivetrain, 0, 0, "LEFT".equals(Constants.Field.STARTING_POSITION) ? -60.0 : 60.0),
            //new DriveRelativeMetersCommand(drivetrain, neutralTraversalDistance, 0),
            new DriveCommand(drivetrain, neutralTraversalDistance, 0, 0),
            //new RotateRelativeDegreesCommand(drivetrain, "LEFT".equals(Constants.Field.STARTING_POSITION) ? -90.0 : 90.0),
            new DriveCommand(drivetrain, 0, 0, "LEFT".equals(Constants.Field.STARTING_POSITION) ? -90.0 : 90.0),
            new IntakeCommand(intakeSubsystem).withTimeout(15),
            //new DriveRelativeMetersCommand(drivetrain, FieldConstants.Fuel.FUEL_BOUND_BOX_LENGTH+1, 0),
            new DriveCommand(drivetrain, FieldConstants.Fuel.FUEL_BOUND_BOX_LENGTH + 1, 0, 0),
            //new DriveRelativeMetersCommand(drivetrain, -FieldConstants.Fuel.FUEL_BOUND_BOX_LENGTH-1, 0),
            new DriveCommand(drivetrain, -1 * FieldConstants.Fuel.FUEL_BOUND_BOX_LENGTH - 1, 0, 0),
            //new DriveRelativeMetersCommand(drivetrain, 0, "LEFT".equals(Constants.Field.STARTING_POSITION) ? neutralTraversalDistance : -neutralTraversalDistance),
            new DriveCommand(drivetrain, 0, "LEFT".equals(Constants.Field.STARTING_POSITION) ? neutralTraversalDistance : -neutralTraversalDistance, 0),
            //new RotateRelativeDegreesCommand(drivetrain, "LEFT".equals(Constants.Field.STARTING_POSITION) ? 30.0 : -30.0),
            new DriveCommand(drivetrain, 0, 0, "LEFT".equals(Constants.Field.STARTING_POSITION) ? 30.0 : -30.0),
            new ShootWithPowerCommand(turretSubsystem, Constants.Turret.ShootConfig.AUTO_SPITTER_SPEED).withTimeout(10)
            );
        
    }
}