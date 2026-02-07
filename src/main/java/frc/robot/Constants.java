package frc.robot;

public class Constants {
    // Controller
    public static final int CONTROLLER_PORT = 0; // Port on the Driverstation

    public class Swerve {
        public static final double MAX_DRIVE_SPEED = 0.5; // Meters per second
    }

    public class Vision {
        public static final String DEFAULT_CAM = "good cam";
        
        public static final double ANGLE_DEADBAND = 2.00; // Degrees
        public static final double ANGLE_CONVERSION_FACTOR = .04; // 1/25
        
        public static final double FOLLOW_SPEED = 0.5; // Meters :P
        public static final double BACKUP_DIST = .7; // Meters

        public static final double DISTANCE_CONVERSION_FACTOR = 0.333; // 1/3
    }

    public class Turret {
        public static final int KICKER_MOTOR_PORT = 0;
        public static final int SHOOTER_MOTOR_PORT = 0;
        public static final int PITCH_MOTOR_PORT = 0;
        public static final int YAW_MOTOR_PORT = 0;
    }
}
