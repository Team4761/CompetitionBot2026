package frc.robot.util;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;

public class SmartVortexMotor {
    private PWMSparkFlex motor;

    public SmartVortexMotor(Builder builder) {
        this.motor = new PWMSparkFlex(builder.port);
    }

    public void setSpeed(double speed) {
        this.motor.set(speed);
    }

    public void stopTurning() {
        this.motor.set(0);
    }

    public static class Builder {
        private int port;
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder port(int port) { this.port = port; return this; }
        
        public SmartVortexMotor build() { return new SmartVortexMotor(this); }
    }
}
