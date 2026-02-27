package frc.robot.util;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SmartVortexMotor {
    private SparkFlex motor;

    public SmartVortexMotor(Builder builder) {
        this.motor = new SparkFlex(builder.canId, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        this.motor.set(speed);
    }

    public void stopTurning() {
        this.motor.set(0);
    }

    public static class Builder {
        private int canId;
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder canId(int canId) { this.canId = canId; return this; }
        public Builder port(int port) { this.canId = port; return this; }
        
        public SmartVortexMotor build() { return new SmartVortexMotor(this); }
    }
}
