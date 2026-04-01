package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import java.util.logging.Logger;

public class SmartKrakenMotor {
    public enum MotorMode {
        CONTINUOUS,
        WRAPPED
    }

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final TalonFX motor;
    private final double gearRatio;
    private double minAngle;
    private double maxAngle;
    private MotorMode mode;
    private boolean coastingEnabled = false;

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
        this.mode = builder.mode;
        this.gearRatio = builder.gearRatio;
    }

    public void setRawSpeedPercent(double speedPercent) {
        // Duty cycle is percent output: -1.0 full reverse, 0.0 stop, 1.0 full forward.
        this.motor.setControl(this.dutyCycleRequest.withOutput(clampDutyCycle(speedPercent)));
    }

    public void setRawSpeed(double speedRPM) {
        this.motor.setControl(new VelocityDutyCycle(speedRPM / 60.0));
    }

    public void setSpeedPercent(double speedPercent) {
        this.setRawSpeedPercent(speedPercent);
    }

    public void setSpeed(double speedRPM) {
        this.setRawSpeed(speedRPM * this.gearRatio);
    }

    public boolean turn(double degrees) {
        double targetAngle = this.getAngle() + degrees;
        if (this.mode == MotorMode.WRAPPED) {
            targetAngle %= 360.0;
        }

        if ((this.minAngle == -1 && this.maxAngle == -1)
            || (targetAngle >= this.minAngle && targetAngle <= this.maxAngle)) {
            this.motor.setControl(this.positionRequest.withPosition(this.toMotorRotations(targetAngle)));
            return true;
        }
        
        else {
            LOGGER.warning(String.format("Tried turning to angle: [%.2f] which is past angle limits min: [%.2f] max: [%.2f]", targetAngle, this.minAngle, this.maxAngle));
            return false;
        }
    }

    public boolean set(double degrees) {
        double targetAngle = degrees;
        if (this.mode == MotorMode.WRAPPED) {
            targetAngle %= 360.0;
        }

        if ((this.minAngle == -1 && this.maxAngle == -1) ||
            (targetAngle >= this.minAngle && targetAngle <= this.maxAngle)) {
            this.motor.setControl(this.positionRequest.withPosition(this.toMotorRotations(targetAngle)));
            return true;
        }
        
        else {
            LOGGER.warning(String.format("Tried setting angle to: [%.2f] which is past angle limits min: [%.2f] max: [%.2f]",
                                            targetAngle, this.minAngle, this.maxAngle));
            return false;
        }
    }

    public double getAngle() {
        return this.toMechanismDegrees(this.motor.getPosition().getValueAsDouble());
    }

    public double getSpeedRPM() {
        return (this.motor.getVelocity().getValueAsDouble() * 60.0) / this.gearRatio;
    }

    public void stopTurning() {
        this.motor.setControl(this.dutyCycleRequest.withOutput(0.0));
        if (!this.coastingEnabled) {
            this.motor.setControl(
                this.positionRequest.withPosition(this.motor.getPosition().getValueAsDouble())
            );
        }
    }

    public void enableCoasting() {
        this.coastingEnabled = true;
        this.motor.setControl(this.dutyCycleRequest.withOutput(0.0));
    }

    public void disableCoasting() {
        this.coastingEnabled = false;
        this.motor.setControl(
            this.positionRequest.withPosition(this.motor.getPosition().getValueAsDouble())
        );
    }

    private double toMotorRotations(double mechanismDegrees) {
        return (mechanismDegrees / 360.0) * this.gearRatio;
    }

    private double toMechanismDegrees(double motorRotations) {
        return (motorRotations * 360.0) / this.gearRatio;
    }

    private static double clampDutyCycle(double output) {
        return Math.max(-1.0, Math.min(1.0, output));
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
        private MotorMode mode;
        private double gearRatio = 1.0;
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder port(int port) { this.port = port; return this; }
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        public Builder outputRange(double minOutput, double maxOutput) {
            // TalonFX duty cycle limits are valid only in the [-1.0, 1.0] range.
            this.minOutput = clampDutyCycle(minOutput);
            this.maxOutput = clampDutyCycle(maxOutput);
            return this;
        }
        public Builder angleLimits(double minAngle, double maxAngle) { this.minAngle = minAngle; this.maxAngle = maxAngle; return this; }
        public Builder mode(MotorMode mode) { this.mode = mode; return this; }
        public Builder gearRatio(double gearRatio) { this.gearRatio = gearRatio; return this; }
        
        public SmartKrakenMotor build() { return new SmartKrakenMotor(this); }
    }
}
