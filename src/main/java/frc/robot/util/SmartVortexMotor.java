package frc.robot.util;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SmartVortexMotor {
    private final SparkFlex motor;
    private final double gearRatio;

    public SmartVortexMotor(Builder builder) {
        this.motor = new SparkFlex(builder.canId, MotorType.kBrushless);
        this.gearRatio = builder.gearRatio;
    }

    public void setRawSpeedPercent(double speed) {
        this.motor.set(clampDutyCycle(speed));
    }

    public void setRawSpeed(double speedRPM) {
        this.motor.getClosedLoopController().setSetpoint(speedRPM, SparkBase.ControlType.kVelocity);
    }

    public void setSpeedPercent(double speed) {
        this.setRawSpeedPercent(speed * this.gearRatio);
    }

    public void setSpeed(double speedRPM) {
        this.setRawSpeed(speedRPM * this.gearRatio);
    }

    public void stopTurning() {
        this.setRawSpeedPercent(0.0);
    }

    private static double clampDutyCycle(double output) {
        return Math.max(-1.0, Math.min(1.0, output));
    }

    public static class Builder {
        private int canId;
        private double gearRatio = 1.0;
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder canId(int canId) { this.canId = canId; return this; }
        public Builder port(int port) { this.canId = port; return this; }
        public Builder gearRatio(double gearRatio) { this.gearRatio = gearRatio; return this; }
        
        public SmartVortexMotor build() { return new SmartVortexMotor(this); }
    }
}
