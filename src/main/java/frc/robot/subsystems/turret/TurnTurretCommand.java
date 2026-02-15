package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


//Command to move a set distance.
public class TurnTurretCommand extends Command{


    private double degreesToRotate;

    public TurnTurretCommand(double degrees)
    {
        this.degreesToRotate = degrees;
    }
    /**
     * <p> All that needs to be initialized.
     * @param deltaX How far the x value is from where the robot is now in meters. Positive x is forward and negative is backward.
     * @param deltaY How far the y value is from where the robot is now in meters. Positive y is left and negative is right.
     * @param deltaRot How much the robot should rotate
     */
    public static Command create(double degreesToRotate)
    {
        return new TurnTurretCommand(degreesToRotate);
    }

    public void turnMotor(double yaw) {
       
        
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean isInterrupted) {
    }
}