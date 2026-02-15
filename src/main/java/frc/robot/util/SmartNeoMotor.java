package frc.robot.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import java.util.logging.Logger;

public class SmartNeoMotor {
    private SparkMaxConfig config = new SparkMaxConfig(); // Default config, will be overridden by builder values
    private RelativeEncoder encoder; // The encoder for the motor, used for feedback in closed-loop control.
    private SparkClosedLoopController PIDController; // The PID controller for the motor, used for closed-loop control of the motor's position.
    private SparkMax motor; // The motor controller.
    private double minAngle; // Minimum angle limit for the motor. Set to -1 for no limit. (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)
    private double maxAngle; // Maximum angle limit for the motor. Set to -1 for no limit. (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)

    private double currentAngle; // Current angle of the motor (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)

    private static final Logger LOGGER = Logger.getLogger(SmartNeoMotor.class.getName()); // Logger for logging warnings and information about the motor's behavior.
    
    /**
     * Constructor for the SmartNeoMotor. This will initialize the motor with the given configuration and set the initial angle to 0.
     * @param builder The builder object containing the configuration for the motor.
     */
    public SmartNeoMotor(Builder builder) {
        this.motor = new SparkMax(builder.port, builder.motorType); // Initialize the motor controller with the specified port and motor type.
        
        this.config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(builder.p, builder.i, builder.d)
            .outputRange(builder.minOutput, builder.maxOutput); // Configure the PID controller with the specified gains and output range from the builder.
        this.motor.configure(this.config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters); // Apply the configuration to the motor controller, resetting to safe parameters and not persisting parameters across power cycles.
        
        this.encoder = this.motor.getEncoder(); // Get the encoder from the motor controller
        this.encoder.setPosition(0); // Set the initial position of the encoder to 0

        this.PIDController = this.motor.getClosedLoopController(); // get the PID controller
    
        this.minAngle = builder.minAngle; // get min angle from builder
        this.maxAngle = builder.maxAngle; // get max angle from builder
    }
    /**
     * Sets the speed of the motor using duty cycle control. This will directly set the output voltage proportion for the motor, ignoring any position control. The speed should be between -1.0 (full reverse) and 1.0 (full forward).
     * @param speed The speed to set the motor to. Should be between -1.0 and 1.0.
     */
    public void setSpeed(double speed) { 
        this.motor.set(speed); 
    }
    
    /**
     * Turns the motor by a specified number of degrees, while respecting the angle limits. If the resulting angle exceeds the limits, it will not turn and will log a warning.
     * @param degrees The number of degrees to turn the motor by. Positive values will turn in one direction [Clockwise?], and negative values will turn in the opposite direction [Counter-Clockwise?].
     */
    public void turn(double degrees) {
        if ((this.minAngle == -1 && this.maxAngle == -1) || // If there are no angle limits, allow any turn.
            ((this.currentAngle + degrees) % 360 >= this.minAngle) && // Check if the resulting angle is greater than or equal to the minimum angle limit AND
            ((this.currentAngle + degrees) % 360 <= this.maxAngle)) { // Check if the resulting angle is less than or equal to the maximum angle limit.
            this.currentAngle += degrees;
            this.currentAngle %= 360; // Keep the current angle within the range of 0 to 360 degrees.
            this.PIDController.setReference(this.currentAngle / 3.6, ControlType.kPosition); // Convert degrees to the motor's position units (assuming 360 degrees corresponds to 100% duty cycle) and set the reference for the PID controller to move the motor to the desired angle.
        } else {
            LOGGER.warning(String.format("Tried turning to angle: [%d] which is past angle limits min: [%d] max: [%d]", 
                                            this.currentAngle, 
                                            this.minAngle, 
                                            this.maxAngle)); // Log a warning if the turn would exceed the angle limits.
        }
    }
    
    /**
     * Returns the current angle of the motor in degrees.
     * @return The current angle of the motor in degrees.    
     */
    public double getAngle() { 
        return this.encoder.getPosition(); 
    }

    /**
     * Stops the motor from turning and holds its current position using the PID controller. This will set the motor output to 0 and set the PID controller's reference to the current position, effectively holding the motor in place.
     */
    public void stopTurning() {
        this.motor.set(0);
        this.PIDController.setReference(
            this.encoder.getPosition(), 
            ControlType.kPosition
        );
    }

    /**
     * Builder class for constructing a SmartNeoMotor with a fluent interface. This allows for easy and readable configuration of the motor's parameters when creating an instance of SmartNeoMotor.
     */
    public static class Builder {
        private int port; // Port ID for the motor controller.
        private MotorType motorType; // Type of motor (e.g., brushless or brushed) for the motor controller.
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
        public Builder motorType(MotorType motorType) { this.motorType = motorType; return this; }
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        public Builder outputRange(double minOutput, double maxOutput) { this.minOutput = minOutput; this.maxOutput = maxOutput; return this; }
        public Builder angleLimits(double minAngle, double maxAngle) { this.minAngle = minAngle; this.maxAngle = maxAngle; return this; }
        
        public SmartNeoMotor build() { return new SmartNeoMotor(this); }
    }
}
