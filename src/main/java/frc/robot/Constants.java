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

        public static final int TRACKED_TAG_ID = 0; 
    }

    public class Climber {
        public static final int CLIMBER_MOTOR_PORT = 0; 
    }

    public class Turret {
        public static final int SPITTER_MOTOR_PORT = 0; // Type: kraken
        public static final int HORIZONTAL_MOTOR_PORT = 0; // Type: kraken
        public static final int VERTICAL_MOTOR_PORT = 0; // Type: kraken

        public static final int SPINDEXER_MOTOR_PORT = 20; // Type: flex/vortex
        public static final int KICKER_MOTOR_PORT = 19; // Type: flex/vortex

        public class Horizontal {
            public static final double ANGLE_TURN_THRESHOLD = 2; // Units: degrees
            public static final double ANGLE_TURN_PERCENTAGE = 0.9; // Units: percent (0-1)
            public static final double MAX_TRACK_STEP_DEGREES = 4.0; // Units: degrees
            public static final double MAX_TRACK_RATE_DEGREES_PER_SEC = 120.0; // Units: degrees/seconds

            public static final double CONVERSION_FACTOR_MtoT = 185 / 28; // Motor rotations -> turret rotations
            public static final double CONVERSION_FACTOR_TtoM = 28 / 185; // Turret rotations -> motor rotations

            public static final double ANGLE_LIM_LEFT = -80.0; // Units: degrees
            public static final double ANGLE_LIM_RIGHT = 80.0; // Units: degrees
        }

        public class Vertical {
            public static final double CONVERSION_FACTOR_MtoH = 325 / 18; // Motor rotations -> hood rotations
            public static final double CONVERSION_FACTOR_HtoM = 18 / 325; // Hood rotations -> motor rotations

            public static final double ANGLE_LIM_LEFT = 25; // Units: degrees
            public static final double ANGLE_LIM_RIGHT = 0; // Units: degrees
        }
    }
}
