package frc.robot.subsystems.swerve.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class DriveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double metersX;
    private final double metersY;
    private final double degrees;
    private static Pose2d start;
    private static PIDController thetaPID;
    private static SwerveRequest.FieldCentric drive;
    public DriveCommand(CommandSwerveDrivetrain drivetrain, double metersX, double metersY, double degrees) {
        this.drivetrain = drivetrain;
        this.metersX = metersX;// * Constants.Swerve.Auto.SWERVE_MULTIPLIER;
        this.metersY = metersY;
        this.degrees = degrees;
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        System.out.println("INITALIZED======================\na\na\na\na\na");
        start = drivetrain.getState().Pose; 
        thetaPID = new PIDController(Constants.Swerve.Auto.ROTATION_KP, 0, 0);
        drive = new SwerveRequest.FieldCentric();
    }

    @Override
    public void execute() {
        System.out.println("Drivin'");
        Pose2d pose = drivetrain.getState().Pose;
        double dx = pose.getX() - start.getX();
        double errorX = metersX - dx;
        double speedX = errorX * 2.0;

        double dy = pose.getY() - start.getY();
        double errorY = metersY - dy;
        double speedY = errorY * 2.0;

        double current = drivetrain.getState().Pose.getRotation().getRadians();
        double target = Math.toRadians(degrees);
        double omega = thetaPID.calculate(current, target);
        //System.out.println(speedX + "\n" + speedY + "\n" + omega);
        drivetrain.setControl(
            drive.withVelocityX(speedX)
                 .withVelocityY(speedY)
                 .withRotationalRate(omega)
         );
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> new SwerveRequest.Idle());
        thetaPID.close();
    }

    @Override
    public boolean isFinished() {
        System.out.println(Math.abs(drivetrain.getState().Pose.getX() - start.getX() - metersX));
        System.out.println(Math.abs(drivetrain.getState().Pose.getY() - start.getY() - metersY ));
        System.out.println(Math.abs(Math.toDegrees(drivetrain.getState().Pose.getRotation().getRadians()) - degrees));
        return ( Math.abs(drivetrain.getState().Pose.getX() - start.getX() - metersX) < 0.05 && 
                 Math.abs(drivetrain.getState().Pose.getY() - start.getY() - metersY) < 0.05 && 
                 Math.abs(Math.toDegrees(drivetrain.getState().Pose.getRotation().getRadians()) - degrees) < 2.0
        );
    }
}
