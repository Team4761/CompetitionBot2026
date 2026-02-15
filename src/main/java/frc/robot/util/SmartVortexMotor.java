package frc.robot.util;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;

public class SmartVortexMotor {
    private PWMSparkFlex motor; // The motor controller.
    private double minAngle; // Minimum angle limit for the motor. Set to -1 for no limit. (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)
    private double maxAngle; // Maximum angle limit for the motor. Set to -1 for no limit. (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)

    private double currentAngle; // Current angle of the motor (Is it in degrees or radians? Assuming degrees for now, but can be changed to radians if needed.)

    /**
     * Constructor for the SmartVortexMotor. This will initialize the motor with the given configuration and set the initial angle to 0.
     * @param builder The builder object containing the configuration for the motor.
     */
    public SmartVortexMotor(Builder builder) {
        this.motor = new PWMSparkFlex(builder.port); // Initialize the motor controller with the specified port and motor type.
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
     * Turns the motor by a specified number of degrees, while respecting the angle limits. If the resulting angle exceeds the limits, it will not turn and will log a warning. (DOES NOT WORK RIGHT NOW)
     * @param degrees The number of degrees to turn the motor by. Positive values will turn in one direction [Clockwise?], and negative values will turn in the opposite direction [Counter-Clockwise?].
     */
    public void turn(double degrees) {
        if ((this.minAngle == -1 && this.maxAngle == -1) || // If there are no angle limits, allow any turn.
            ((this.currentAngle + degrees) % 360 >= this.minAngle) && // Check if the resulting angle is greater than or equal to the minimum angle limit AND
            ((this.currentAngle + degrees) % 360 <= this.maxAngle)) { // Check if the resulting angle is less than or equal to the maximum angle limit.
            this.currentAngle += degrees;
            this.currentAngle %= 360; // Keep the current angle within the range of 0 to 360 degrees.
            // TODO: Make it
        }
    }
    

    /**
     * Stops the motor from turning by setting the speed to 0. This will immediately stop the motor's movement.
     */
    public void stopTurning() {
        this.motor.set(0);
    }

    /**
     * Builder class for constructing a SmartNeoMotor with a fluent interface. This allows for easy and readable configuration of the motor's parameters when creating an instance of SmartNeoMotor.
     */
    public static class Builder {
        private int port; // Port ID for the motor controller.
        private double minAngle; // Minimum angle limit for the motor (assuming degrees, between 0 and 360). Set to -1 for no limit.
        private double maxAngle; // Maximum angle limit for the motor (assuming degrees, between 0 and 360). Set to -1 for no limit.
        
        public static Builder newInstance() { return new Builder(); }
        
        public Builder() {}

        public Builder port(int port) { this.port = port; return this; }
        public Builder angleLimits(double minAngle, double maxAngle) { this.minAngle = minAngle; this.maxAngle = maxAngle; return this; }
        
        public SmartVortexMotor build() { return new SmartVortexMotor(this); }
    }
}
