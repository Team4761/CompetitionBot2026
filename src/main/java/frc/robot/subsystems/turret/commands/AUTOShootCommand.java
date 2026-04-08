package frc.robot.subsystems.turret.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.turret.TurretSubsystem;

public class AUTOShootCommand extends SequentialCommandGroup {
    private static double getPower(double dist) {
        Double[] powerBandRanges = {1.36, 1.85, 2.496, 3.118, 3.643, 4.123, 4.562, 4.991, 5.055}; // low < dist <= high
        
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

    private static double calculateAngle(double dist) {
        Double[] distPoints = {1.36, 1.49, 1.595, 1.754, 1.85, 1.967, 2.192, 2.338, 2.496, 2.623, 2.738, 2.841, 2.911, 3.009, 3.118, 3.207, 3.296, 3.407, 3.531, 3.643, 3.751, 3.861, 3.959, 4.123, 4.256, 4.386, 4.513, 4.562, 4.641, 4.767, 4.826, 4.978, 4.991, 5.055};
        Double[] anglePoints = {22.0, 22.0, 22.0, 25.03, 26.51, 22.0, 26.11, 27.36, 30.21, 27.08, 28.16, 30.33, 34.31, 36.08, 38.07, 32.0, 35.05, 36.53, 38.01, 38.01, 31.12, 32.32, 33.06, 37.67, 36.13, 36.13, 39.09, 41.0, 35.73, 35.73, 38.07, 42.45, 35.45, 36.82};
        
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

    public AUTOShootCommand(TurretSubsystem sub, DoubleSupplier distSupplier) {
        super(
            new PrintCommand(Double.toString(calculateAngle(distSupplier.getAsDouble()))),
            new InstantCommand(() -> sub.verticalMotor.set(calculateAngle(distSupplier.getAsDouble())), sub),
            new ShootWithPowerCommand(sub, () -> getPower(distSupplier.getAsDouble()))
        );
    }
}
