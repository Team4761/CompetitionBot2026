package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

