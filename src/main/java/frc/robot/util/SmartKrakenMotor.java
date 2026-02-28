package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.logging.Logger;

public class SmartKrakenMotor {
    private static final double NO_LIMIT = -1.0;
    private static final double DEGREES_PER_ROTATION = 360.0;

    public enum MotorMode {
        CONTINUOUS,
        WRAPPED
    }

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final TalonFX motor;
    private double minAngle;
    private double maxAngle;
    private MotorMode mode;
    private final double motorRotationsPerMechanismRotation;

    private double currentAngle = 0.0;

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
        this.motorRotationsPerMechanismRotation = builder.motorRotationsPerMechanismRotation;

        if (this.motorRotationsPerMechanismRotation <= 0.0) {
            throw new IllegalArgumentException("motorRotationsPerMechanismRotation must be > 0");
        }

        if (!hasNoLimits() && this.minAngle > this.maxAngle) {
            LOGGER.warning(String.format(
                "Reversed angle limits provided, swapping min/max [%.2f, %.2f]",
                this.minAngle,
                this.maxAngle
            ));
            double temp = this.minAngle;
            this.minAngle = this.maxAngle;
            this.maxAngle = temp;
        }
    }

    public void setSpeedPercent(double speedPercent) {
        this.motor.setControl(this.dutyCycleRequest.withOutput(speedPercent));
    }

    public void setSpeed(double speedRPM) {
        this.motor.setControl(new VelocityDutyCycle(speedRPM / 60.0));
    }

    public boolean turn(double degrees) {
        double targetAngle = this.currentAngle + degrees;
        if (this.mode == MotorMode.WRAPPED) {
            targetAngle = normalizeWrappedAngle(targetAngle);
        }

        if (isWithinLimits(targetAngle)) {
            this.currentAngle = targetAngle;
            this.motor.setControl(this.positionRequest.withPosition(angleToMotorRotations(this.currentAngle)));
            return true;
        } else {
            LOGGER.warning(String.format("Tried turning to angle: [%.2f] which is past angle limits min: [%.2f] max: [%.2f]", 
                                            targetAngle, this.minAngle, this.maxAngle));
            return false;
        }
    }

    public boolean set(double degrees) {
        double targetAngle = degrees;
        if (this.mode == MotorMode.WRAPPED) {
            targetAngle = normalizeWrappedAngle(targetAngle);
        }

        if (isWithinLimits(targetAngle)) {
            this.currentAngle = targetAngle;
            this.motor.setControl(this.positionRequest.withPosition(angleToMotorRotations(this.currentAngle)));
            return true;
        } else {
            LOGGER.warning(String.format("Tried setting angle to: [%.2f] which is past angle limits min: [%.2f] max: [%.2f]",
                                            targetAngle, this.minAngle, this.maxAngle));
            return false;
        }
    }

    public double getAngle() {
        return this.motor.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION / this.motorRotationsPerMechanismRotation;
    }

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
        private double minAngle = NO_LIMIT;
        private double maxAngle = NO_LIMIT;
        private MotorMode mode = MotorMode.CONTINUOUS;
        private double motorRotationsPerMechanismRotation = 1.0;
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder port(int port) { this.port = port; return this; }
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        public Builder outputRange(double minOutput, double maxOutput) { this.minOutput = minOutput; this.maxOutput = maxOutput; return this; }
        public Builder angleLimits(double minAngle, double maxAngle) { this.minAngle = minAngle; this.maxAngle = maxAngle; return this; }
        public Builder mode(MotorMode mode) { this.mode = mode; return this; }
        public Builder motorRotationsPerMechanismRotation(double ratio) { this.motorRotationsPerMechanismRotation = ratio; return this; }
        
        public SmartKrakenMotor build() { return new SmartKrakenMotor(this); }
    }

    private boolean hasNoLimits() {
        return this.minAngle == NO_LIMIT && this.maxAngle == NO_LIMIT;
    }

    private boolean isWithinLimits(double targetAngle) {
        return hasNoLimits() || (targetAngle >= this.minAngle && targetAngle <= this.maxAngle);
    }

    private double angleToMotorRotations(double angleDegrees) {
        return (angleDegrees / DEGREES_PER_ROTATION) * this.motorRotationsPerMechanismRotation;
    }

    private double normalizeWrappedAngle(double angleDegrees) {
        double normalized = angleDegrees % DEGREES_PER_ROTATION;
        return normalized < 0 ? normalized + DEGREES_PER_ROTATION : normalized;
    }
}
