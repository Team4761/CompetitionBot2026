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
    private SparkMaxConfig config = new SparkMaxConfig();
    private RelativeEncoder encoder;
    private SparkClosedLoopController PIDController;
    private SparkMax motor;
    private double minAngle;
    private double maxAngle;

    private double currentAngle;

    private static final Logger LOGGER = Logger.getLogger(SmartNeoMotor.class.getName());
    
    public SmartNeoMotor(Builder builder) {
        this.motor = new SparkMax(builder.port, builder.motorType);
        
        this.config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(builder.p, builder.i, builder.d)
            .outputRange(builder.minOutput, builder.maxOutput);
        this.motor.configure(this.config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
        
        this.encoder = this.motor.getEncoder();
        this.encoder.setPosition(0);

        this.PIDController = this.motor.getClosedLoopController();
    
        this.minAngle = builder.minAngle;
        this.maxAngle = builder.maxAngle;
    }

    public void setSpeed(double speed) { this.motor.set(speed); }
    public void turn(double degrees) {
        if ((this.minAngle == -1 && this.maxAngle == -1) 
            || (this.currentAngle + degrees) % 360 >= this.minAngle 
                && (this.currentAngle + degrees) % 360 <= this.maxAngle) {
            this.currentAngle += degrees;
            this.currentAngle %= 360;
            this.PIDController.setReference(this.currentAngle / 3.6, ControlType.kPosition);
        } else {
            LOGGER.warning(String.format("Tried turning to angle: [%d] which is past angle limits min: [%d] max: [%d]", 
                                            this.currentAngle, 
                                            this.minAngle, 
                                            this.maxAngle));
        }
    }
    public double getAngle() { return this.encoder.getPosition(); }
    public void stopTurning() {
        this.motor.set(0);
        this.PIDController.setReference(
            this.encoder.getPosition(), 
            ControlType.kPosition
        );
    }

    public static class Builder {
        private int port;
        private MotorType motorType;
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
        public Builder motorType(MotorType motorType) { this.motorType = motorType; return this; }
        public Builder PID(double p, double i, double d) { this.p = p; this.i = i; this.d = d; return this; }
        public Builder outputRange(double minOutput, double maxOutput) { this.minOutput = minOutput; this.maxOutput = maxOutput; return this; }
        public Builder angleLimits(double minAngle, double maxAngle) { this.minAngle = minAngle; this.maxAngle = maxAngle; return this; }
        
        public SmartNeoMotor build() { return new SmartNeoMotor(this); }
    }
}
