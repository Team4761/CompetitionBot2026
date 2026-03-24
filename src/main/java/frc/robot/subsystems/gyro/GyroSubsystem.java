package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class GyroSubsystem extends SubsystemBase{
    // Make the code aware there should be 2 motors
    private final Pigeon2 pigeon;

    // Tell the code what those motor are/should be like and were to find them
    public GyroSubsystem() {
        this.pigeon = new Pigeon2(Constants.Gyro.PIGEON_ID);
    }

    public double getYaw() { return this.pigeon.getYaw().getValueAsDouble(); }
    public double getPitch() { return this.pigeon.getPitch().getValueAsDouble(); }
    public double getRoll() { return this.pigeon.getRoll().getValueAsDouble(); }

    public double getXAcc() { return this.pigeon.getAccelerationX().getValueAsDouble(); }
    public double getYAcc() { return this.pigeon.getAccelerationY().getValueAsDouble(); }
    public double getZAcc() { return this.pigeon.getAccelerationZ().getValueAsDouble(); }

    public Rotation3d getR3d() { return this.pigeon.getRotation3d(); }
    public Rotation2d getR2d() { return this.pigeon.getRotation2d(); }
}
