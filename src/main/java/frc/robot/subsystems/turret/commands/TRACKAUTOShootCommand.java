package frc.robot.subsystems.turret.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.commands.DriveCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TRACKAUTOShootCommand extends SequentialCommandGroup {
    private static Double[] powerBandRanges = {1.36, 1.85, 2.496, 3.118, 3.643, 4.123, 4.562, 4.991, 5.055}; // low < dist <= high
    private static Double[] anglePoints = {22.0, 22.0, 22.0, 25.03, 26.51, 22.0, 26.11, 27.36, 30.21, 27.08, 28.16, 30.33, 34.31, 36.08, 38.07, 32.0, 35.05, 36.53, 38.01, 38.01, 31.12, 32.32, 33.06, 37.67, 36.13, 36.13, 39.09, 41.0, 35.73, 35.73, 38.07, 42.45, 35.45, 36.82};
    private static Double[] distPoints = {1.36, 1.49, 1.595, 1.754, 1.85, 1.967, 2.192, 2.338, 2.496, 2.623, 2.738, 2.841, 2.911, 3.009, 3.118, 3.207, 3.296, 3.407, 3.531, 3.643, 3.751, 3.861, 3.959, 4.123, 4.256, 4.386, 4.513, 4.562, 4.641, 4.767, 4.826, 4.978, 4.991, 5.055};
    private static Double[] timePoints = {};

    private static double getTime(double dist) {
        if (dist <= timePoints[0]) {
            return timePoints[0];
        } else if (dist >= timePoints[timePoints.length - 1]) {
            return timePoints[timePoints.length - 1];
        }

        int index = Arrays.binarySearch(timePoints, dist);
        if (index >= 0) {
            return timePoints[index];
        } else {
            index = -index - 1; // Get the insertion point

            Double leftDist = timePoints[index - 1];
            Double rightDist = timePoints[index];

            Double leftTime = timePoints[index - 1];
            Double rightTime = timePoints[index];
            
            return leftTime + (rightTime - leftTime) * (dist - leftDist) / (rightDist - leftDist);
        }
    }
    
    private static double getPower(double dist) {
        if (dist <= powerBandRanges[0]) {
            return 3750;
        } else if (dist >= powerBandRanges[powerBandRanges.length - 1]) {
            return 3750 + powerBandRanges.length * 250;
        }

        int index = Arrays.binarySearch(powerBandRanges, dist);
        if (index < 0) {
            index = -index - 1; // insertion point
        }
        return 3750 + (index - 1) * 250;
    }

    private static double getLaunchAngle(double dist) {
        if (dist <= distPoints[0]) {
            return anglePoints[0];
        } else if (dist >= distPoints[distPoints.length - 1]) {
            return anglePoints[anglePoints.length - 1];
        }

        int index = Arrays.binarySearch(distPoints, dist);
        if (index >= 0) {
            return anglePoints[index];
        } else {
            index = -index - 1; // Get the insertion point

            Double leftDist = distPoints[index - 1];
            Double rightDist = distPoints[index];

            Double leftAngle = anglePoints[index - 1];
            Double rightAngle = anglePoints[index];
            
            return leftAngle + (rightAngle - leftAngle) * (dist - leftDist) / (rightDist - leftDist);
        }
    }

    private static Pose2d getFuturePose(CommandSwerveDrivetrain swerve, double time) {
        ChassisSpeeds speeds = swerve.getState().Speeds;
        Transform2d velocity = new Transform2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond)
        );
        velocity = velocity.times(time);

        Pose2d futurePose = swerve.getState().Pose.plus(velocity);

        return futurePose;
    }

    private static Command pointToHub(CommandSwerveDrivetrain swerve, Pose2d pose) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        double hubX = alliance == DriverStation.Alliance.Blue ? FieldConstants.Hub.HUB_BLUE_X : FieldConstants.Hub.HUB_RED_X;
        double hubY = alliance == DriverStation.Alliance.Blue ? FieldConstants.Hub.HUB_BLUE_Y : FieldConstants.Hub.HUB_RED_Y;

        double angleToHub = Math.toDegrees(Math.atan2(hubY - pose.getY(), hubX - pose.getX()));
        
        return new DriveCommand(swerve, 0, 0, angleToHub);
    }

    public TRACKAUTOShootCommand(TurretSubsystem turret, CommandSwerveDrivetrain swerve, DoubleSupplier distSupplier) {
        super(
            new InstantCommand(() -> turret.setLaunchAngleDegrees(getLaunchAngle(distSupplier.getAsDouble())), turret),
            pointToHub(swerve, getFuturePose(swerve, getTime(distSupplier.getAsDouble()))),
            new ShootWithPowerCommand(turret, () -> getPower(distSupplier.getAsDouble()))
        );
    }
}
