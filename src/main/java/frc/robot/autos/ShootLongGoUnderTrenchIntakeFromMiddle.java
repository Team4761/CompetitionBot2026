package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.JostleCommand;
import frc.robot.subsystems.turret.ShootAtAngleAtSpeedCommand;
import frc.robot.subsystems.turret.TurretSubsystem;


public class ShootLongGoUnderTrenchIntakeFromMiddle extends SequentialCommandGroup{
    
    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public ShootLongGoUnderTrenchIntakeFromMiddle(IntakeSubsystem intakeSubsystem,CommandSwerveDrivetrain drivetrain, TurretSubsystem turretSubsystem) {

        final var idle = new SwerveRequest.Idle();
        
        // addCommands(
        //     new DoNothingCommand(),
        //     /*check if 
        //         red team ->
        //         if sees april tag 10 rotated at -45 degrees around the vertical axis(applies to all other angles) and/or 11 at 45
        //         or 
        //         if sees april tag 9 at 45 and 8 at -45

        //         blue team ->
        //         if sees april tag 26 at -45 degrees and/or 27 at 45
        //         or 
        //         if sees april tag 25 at 45 and 24 at -45

        //          make corrections so it sees those tags at those angles
        //     */
        //     new ShootAtAngleAtSpeedCommand(turretSubsystem, 0.0, 1.0).withTimeout(4),
        //     drivetrain.runOnce(drivetrain::seedFieldCentric),
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.0)
        //             .withVelocityY(0.0)
        //             .withRotationalRate(0.5)
        //     ).withTimeout(1.0),//this time may need to be tweaked to be exactly 45
        //     drivetrain.applyRequest(() -> idle).withTimeout(0.1),
        //     /* check if 
        //         previos april tag was 10 or 11
        //             sees april tag 12 at at 0 degrees around the vertical axis(applies to all other angles)
        //         previos tag was 8 or 9
        //             sees april tag 7 at 0 
        //         previos tag was 26 or 27
        //             sees april tag 28 at 0
        //         previos tag was 25 or 24
        //             sees tag 23 at 0
                
        //             make corrections if the april tag is visible but not a 0
        //     */
        //     drivetrain.runOnce(drivetrain::seedFieldCentric),
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(AUTO_SPEED_MPS)
        //             .withVelocityY(0.0)
        //             .withRotationalRate(0.0)
        //     ).withTimeout(10.0),//this time may need to be tweaked
        //     drivetrain.applyRequest(() -> idle).withTimeout(0.1),
        //     Commands.parallel(
        //         new IntakeCommand(intakeSubsystem).withTimeout(10),
        //         drivetrain.runOnce(drivetrain::seedFieldCentric),
        //         drivetrain.applyRequest(() ->
        //             drive.withVelocityX(AUTO_SPEED_MPS)
        //                 .withVelocityY(0.0)
        //                 .withRotationalRate(0.2)
        //     ).withTimeout(10.0)),
        //     drivetrain.applyRequest(() -> idle).withTimeout(0.1),
        //     new JostleCommand(intakeSubsystem)
        // );
    }
}
