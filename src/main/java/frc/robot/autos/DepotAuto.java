package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.autos.ExtendDownMoveAndGather;
import frc.robot.Constants;

public class DepotAuto extends SequentialCommandGroup{

    private static final double AUTO_SPEED_MPS =
        0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public DepotAuto(IntakeSubsystem intakeSubsystem,CommandSwerveDrivetrain drivetrain){

    }

        final var idle = new SwerveRequest.Idle();

        //theoretically, if distances were arbitrary, this would work fine, assuming we start in the center
        //IMPORTANT: we need to calcutlate how many seconds it needs to move, and then create variations.
        //also using a constructor instead of writing the command every time would be nice
        switch (Constants.Field.STARTING_POSITION){
            case "CENTER": {
                addCommands(
                    new DoNothingCommand(),//as a buffer so that the thing that happened las tyear doesent happen
                    drivetrain.runOnce(drivetrain::seedFieldCentric),
                    //right here, we need to add a conditional to determine which starting slot our robot is in
                    //instead of this, we could possibly check and move to april tag 9 (red) or april tag 25 (blue)
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(AUTO_SPEED_MPS).withVelocityY(0.0).withRotationalRate(0.0)).withTimeout(4.0),//temp value
                    drivetrain.applyRequest(() -> idle).withTimeout(0.1), //i don't know if this is necessary, but i added a small buffer in between each movement just in case
                    drivetrain.applyRequest(() -> 
                        drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(-90.0)).withTimeout(2.0),
                    drivetrain.applyRequest(() -> idle).withTimeout(0.1),
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(AUTO_SPEED_MPS).withVelocityY(0.0).withRotationalRate(0.0)).withTimeout(6.0),//temp value
                    drivetrain.applyRequest(() -> idle).withTimeout(0.1),
                    drivetrain.applyRequest(() -> 
                        drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(-90.0)).withTimeout(2.0),
                    drivetrain.applyRequest(() -> idle).withTimeout(0.1),
                    new ExtendDownMoveAndGather(intakeSubsystem, drivetrain)
                );
            }
            case "LEFT": {

            }
            case "RIGHT": {

            }
        }
    }
}
