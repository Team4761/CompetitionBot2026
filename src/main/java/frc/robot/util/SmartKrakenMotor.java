package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.logging.Logger;

public class SmartKrakenMotor {
    private final TalonFXConfiguration config = new TalonFXConfiguration(); // Default config, will be overridden by builder values
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0); // Duty Cycle with set points.
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0); // Output voltage proportion for Duty Cycle.
    private final TalonFX motor; // The motor controller.
    private double minAngle; // Minimum angle limit for the motor. Set to -1 for no limit. (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)
    private double maxAngle; // Maximum angle limit for the motor. Set to -1 for no limit. (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)

    private double currentAngle; // Current angle of the motor (assuming radians)

    private static final Logger LOGGER = Logger.getLogger(SmartKrakenMotor.class.getName()); // Logger for logging warnings and information about the motor's behavior.
    
    /**
     * Constructor for the SmartKrakenMotor. This will initialize the motor with the given configuration and set the initial angle to 0.
     * @param builder The builder object containing the configuration for the motor.
     */
    public SmartKrakenMotor(Builder builder) {
        this.motor = new TalonFX(builder.port);
        
        this.config.Slot0.kP = builder.p; // Proportional gain for the PID controller.
        this.config.Slot0.kI = builder.i; // Integral gain for the PID controller.
        this.config.Slot0.kD = builder.d; // Derivative gain for the PID controller.
        this.config.MotorOutput.PeakForwardDutyCycle = builder.maxOutput; // Maximum forward output for the motor.
        this.config.MotorOutput.PeakReverseDutyCycle = builder.minOutput; // Maximum reverse output for the motor.
        this.motor.getConfigurator().apply(this.config); // Apply the configuration to the motor controller.
        
        this.motor.setPosition(0.0); // Set the initial position of the motor to 0.
    
        this.minAngle = builder.minAngle; // Set the minimum angle limit from the builder.
        this.maxAngle = builder.maxAngle; // Set the maximum angle limit from the builder.
    }

    /**
     * Sets the speed of the motor using duty cycle control. This will directly set the output voltage proportion for the motor, ignoring any position control. The speed should be between -1.0 (full reverse) and 1.0 (full forward).
     * @param speed The speed to set the motor to. Should be between -1.0 and 1.0.
     */
    public void setSpeed(double speed) { 
        this.motor.setControl(this.dutyCycleRequest.withOutput(speed)); 
    }

    /**
     * Turns the motor by a specified number of degrees, while respecting the angle limits. If the resulting angle exceeds the limits, it will not turn and will log a warning.
     * @param degrees The number of degrees to turn the motor. Positive values will turn in one direction [Clockwise?], and negative values will turn in the opposite direction [Counter-Clockwise?].
     */
    public void turn(double degrees) {
        if ((this.minAngle == -1 && this.maxAngle == -1) || // If there are no angle limits, allow any turn.
            ((this.currentAngle + degrees) % 360 >= this.minAngle) && // Check if the resulting angle is greater than or equal to the minimum angle limit AND
            ((this.currentAngle + degrees) % 360 <= this.maxAngle)) { // Check if the resulting angle is less than or equal to the maximum angle limit.
                this.currentAngle += degrees;
                this.currentAngle %= 360; // Keep the current angle within the range of 0 to 360 degrees.
            this.motor.setControl(this.positionRequest.withPosition(this.currentAngle / 3.6)); // Convert degrees to the motor's position units (assuming 360 degrees corresponds to 100% duty cycle).
        } else {
            LOGGER.warning(String.format("Tried turning to angle: [%d] which is past angle limits min: [%d] max: [%d]", 
                                            this.currentAngle, 
                                            this.minAngle, 
                                            this.maxAngle)); // Log a warning if the turn would exceed the angle limits.
        }
    }
    /**
     * Gets the current angle of the motor in degrees. This is calculated based on the motor's position and the assumption that 360 degrees corresponds to a certain number of position units (in this case, 100% duty cycle). The angle is returned in the range of 0 to 360 degrees.
     * @return The current angle of the motor in ???
     */
    public double getAngle() { 
        return this.motor.getPosition().getValueAsDouble(); 
    }

    /**
     * Stops the motor from turning by setting the duty cycle output to 0 and updating the position control to hold the current angle. This will effectively stop any movement of the motor and maintain its current position.
     */
    public void stopTurning() {
        this.motor.setControl(this.dutyCycleRequest.withOutput(0.0));
        this.motor.setControl(
            this.positionRequest.withPosition(this.motor.getPosition().getValueAsDouble())
        );
    }
    /**
     * Builder class for constructing a SmartKrakenMotor with a fluent interface. This allows for easy and readable configuration of the motor's parameters such as PID values, output range, and angle limits. The builder pattern is used to create an instance of SmartKrakenMotor with the specified configuration.
     */
    public static class Builder {
        private int port; // Port ID
        private double p; // Proportional gain for the PID controller.
        private double i; // Integral gain for the PID controller.
        private double d; // Derivative gain for the PID controller.
        private double minOutput; // Minimum output for the motor (as a duty cycle, between -1.0 and 0.0).
        private double maxOutput; // Maximum output for the motor (as a duty cycle, between 0.0 and 1.0).
        private double minAngle; // Minimum angle limit for the motor (assuming degrees, between 0 and 360). Set to -1 for no limit.
        private double maxAngle; // Maximum angle limit for the motor (assuming degrees, between 0 and 360). Set to -1 for no limit.
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder port(int port) { this.port = port; return this; }
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        public Builder outputRange(double minOutput, double maxOutput) { this.minOutput = minOutput; this.maxOutput = maxOutput; return this; }
        public Builder angleLimits(double minAngle, double maxAngle) { this.minAngle = minAngle; this.maxAngle = maxAngle; return this; }
        
        public SmartKrakenMotor build() { return new SmartKrakenMotor(this); }
    }
}
