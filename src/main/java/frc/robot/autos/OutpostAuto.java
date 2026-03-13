package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.commandGroups.TurretTrackCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class OutpostAuto extends SequentialCommandGroup{

    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public OutpostAuto(TurretSubsystem turret, VisionSubsystem vision, CommandSwerveDrivetrain drivetrain){

        final var idle = new SwerveRequest.Idle();
        
        addCommands(
            new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
            drivetrain.runOnce(drivetrain::seedFieldCentric), // reset field heading// driv
            new InstantCommand(() -> turret.setHorizontalMotor(-80)).withTimeout(1), // this set command could use side hub AprilTags
            //new ShootAtAngleAtSpeedCommand(turret, 0, 1).withTimeout(4),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-1.0 * AUTO_SPEED_MPS)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
            ).withTimeout(3),
            drivetrain.applyRequest(() -> idle).withTimeout(0.1),
            new DoNothingCommand().withTimeout(6), // buffer to make sure we are in position before we start tracking again
            new InstantCommand(() -> turret.setHorizontalMotor(-Math.atan2(FieldConstants.Field.ALLIANCE_ZONE_LENGTH/2.0, FieldConstants.Field.ALLIANCE_ZONE_WIDTH/2.0))).withTimeout(1), // currently it could see the april tags on the front side of the hub
            new ShootCommand(turret).withTimeout(6)       
        );
    }
}
