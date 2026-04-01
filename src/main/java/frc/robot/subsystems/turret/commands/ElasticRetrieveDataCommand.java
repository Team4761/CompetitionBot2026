package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * This command is a command for the Elastic Dashboard: it retrieves the horizontal & vertical encoder values for the turret
*/
public class ElasticRetrieveDataCommand extends InstantCommand{
    public ElasticRetrieveDataCommand(TurretSubsystem sub) {
        super(() -> {
            SmartDashboard.putNumber("TURRET HORIZONTAL ANGLE", sub.horizontalMotor.getAngle());
            SmartDashboard.putNumber("TURRET VERTICAL ANGLE", sub.verticalMotor.getAngle());
        });
    }
}

