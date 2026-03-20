package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.baseCommands.DoNothingCommand;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ShootCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
//import edu.wpi.first.math.util.Units;

public class IntakeFromMiddleReturnShootLEFT10s extends SequentialCommandGroup {
    /*
    when startingint the left starting position under the trench this will go forward intake 
    from the middle then it will reverse back to starting pos it will rotate 45 degrees then 
    move into position to shoot
     */
    public IntakeFromMiddleReturnShootLEFT10s(IntakeSubsystem intakeSubsystem,TurretSubsystem turretSubsystem, CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new DoNothingCommand(),//buffer
            //new driveForwardCommand(drivetrain, Units.inchesToMeters((143.5 + (2.0(143.5 - 120.0))))),//move forward (divide by 100 becuase those values ar cm)
            //new turnCommand(drivetrain, left 90),//trun 90 so it can get balls
            new IntakeCommand(intakeSubsystem),//start intake
            //new driveForwardCommand(drivetrain, Units.inchesToMeters(90.95)),//move forward to pick up some balls
            new IntakeCommand(intakeSubsystem).withTimeout(0.1),//stop intake
            //new driveForwardCommand(drivetrain, -1 * Units.inchesToMeters((90.95))),//back up
            //new turnCommand(drivetrain, right 90),////turn right
            //new driveForwardCommand(drivetrain, -1 * Units.inchesToMeters(((143.5 + (2.0(143.5 - 120.0)))))),//back up
            //new turnCommand(drivetrain, left 45),//turn so it can face the hub
            //new strafeLeftCommand(drivetrain, Units.inchesToMeters(Math.sqrt(Math.pow(((181.56 - (143.5 - 120.0))/2), 2)) + Math.pow((158.32/2), 2)))),//move to postion
            new ShootCommand(turretSubsystem).withTimeout(11)//fire!!!!!!!!
        );
    }
}
