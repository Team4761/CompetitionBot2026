package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.logging.Logger;

public class SmartKrakenMotor {
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final TalonFX motor;
    private double minAngle;
    private double maxAngle;

    private double currentAngle;

    private static final Logger LOGGER = Logger.getLogger(SmartKrakenMotor.class.getName());
    
    public SmartKrakenMotor(Builder builder) {
        this.motor = new TalonFX(builder.port);
        
        this.config.Slot0.kP = builder.p;
        this.config.Slot0.kI = builder.i;
        this.config.Slot0.kD = builder.d;
        this.config.MotorOutput.PeakForwardDutyCycle = builder.maxOutput;
        this.config.MotorOutput.PeakReverseDutyCycle = builder.minOutput;
        this.motor.getConfigurator().apply(this.config);
        
        this.motor.setPosition(0.0);
    
        this.minAngle = builder.minAngle;
        this.maxAngle = builder.maxAngle;
    }

    public void setSpeed(double speed) { this.motor.setControl(this.dutyCycleRequest.withOutput(speed)); }
    public void turn(double degrees) {
        if ((this.minAngle == -1 && this.maxAngle == -1) 
            || (this.currentAngle + degrees) % 360 >= this.minAngle 
                && (this.currentAngle + degrees) % 360 <= this.maxAngle) {
            this.currentAngle += degrees;
            this.currentAngle %= 360;
            this.motor.setControl(this.positionRequest.withPosition(this.currentAngle / 3.6));
        } else {
            LOGGER.warning(String.format("Tried turning to angle: [%d] which is past angle limits min: [%d] max: [%d]", 
                                            this.currentAngle, 
                                            this.minAngle, 
                                            this.maxAngle));
        }
    }
    public double getAngle() { return this.motor.getPosition().getValueAsDouble(); }
    public void stopTurning() {
        this.motor.setControl(this.dutyCycleRequest.withOutput(0.0));
        this.motor.setControl(
            this.positionRequest.withPosition(this.motor.getPosition().getValueAsDouble())
        );
    }

    public static class Builder {
        private int port;
        private double p;
        private double i;
        private double d;
        private double minOutput;
        private double maxOutput;
        private double minAngle;
        private double maxAngle;
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder port(int port) { this.port = port; return this; }
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        public Builder outputRange(double minOutput, double maxOutput) { this.minOutput = minOutput; this.maxOutput = maxOutput; return this; }
        public Builder angleLimits(double minAngle, double maxAngle) { this.minAngle = minAngle; this.maxAngle = maxAngle; return this; }
        
        public SmartKrakenMotor build() { return new SmartKrakenMotor(this); }
    }
}
