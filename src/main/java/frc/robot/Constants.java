package frc.robot;

public class Constants {
    public static final double SWERVE_MAX_ANGULAR_ACCELERATION = -1; // placeholder value (in rad/s^2)
    public static final double SWERVE_MAX_ANGULAR_VELOCITY = -1; // placeholder value (in rad/s)
    public static final double WHEEL_RADIUS = 0.04955; // from 2025 Competition Bot Code (in meters)
    public static final double SWERVE_DRIVE_MOTOR_GEAR_RATIO = 6.12; // from 2025 Competition Bot Code (in i don't know)

    //LEDS
    public static final int LEDS_PORT = 0;                 // PWM Port.
    public static final int LEDS_NUMBER_OF_LEDS = 50;      // 10x5 as a test for rick roll, real size is 32x8
    public static final int LEDS_WIDTH = 10;               // change to 32 when the compressor is fixed
    public static final int LEDS_HEIGHT = 5;               // change to 8 when the compressor is fixed
}
