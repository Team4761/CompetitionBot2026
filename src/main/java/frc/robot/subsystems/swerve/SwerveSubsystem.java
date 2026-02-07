package frc.robot.subsystems.swerve;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
    double maximumSpeed = Units.feetToMeters(4.5);
    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive swerveDrive;

    /** Creates a new ExampleSubsystem. */
    public SwerveSubsystem() {
        try
        {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            // TODO: Remove duplicate ID values, stored in both json and constants file
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Swerve.MAX_DRIVE_SPEED);
            swerveDrive.useExternalFeedbackSensor();
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX  Translation in the X direction.
     * @param translationY  Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
            swerveDrive.drive(new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                false,
                false);
        });
    }

    /**
     * Creates a command to reset the gyro of the swerve drive to zero.
     * This is useful for recalibrating the robot's orientation during a match.
     *
     * @return A command that, when executed, resets the gyro to zero.
     */
    public Command reZeroCommand() {
        return runOnce(() -> {
            this.swerveDrive.zeroGyro();
        });
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }
}