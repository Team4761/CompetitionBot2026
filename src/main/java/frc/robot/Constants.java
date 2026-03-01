package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Map;

public class Constants {
    // Controller
    public static final int CONTROLLER_PORT = 0; // Port on the Driverstation

    public static final String TEAM = "BLUE"; // Type: enum("BLUE", "RED")

    public class Controller {
        public static final double TRANSLATION_INPUT_DEADBAND = 0.10;
        public static final double ROTATION_INPUT_DEADBAND = 0.12;
        public static final double TURRET_INPUT_DEADBAND = 0.10;
        public static final double ROTATION_SLEW_RATE_RAD_PER_SEC_SQ = 3.0;
        public static final double TEST_VORTEX_OUTPUT = 0.20;
        public static final double TEST_KRAKEN_OUTPUT = 0.20;
    }

    public class Swerve {
        public static final double MAX_DRIVE_SPEED = 0.5; // Meters per second
    }

    public class Gyro {
        public static final int PIGEON_ID = 0; // Type: pigeon2 [FIXME]
    }

    public class Vision {
        public static final String DEFAULT_CAM = "good cam";
        
        public static final double ANGLE_DEADBAND = 2.00; // Type: Degrees
        public static final double ANGLE_CONVERSION_FACTOR = .04; // 1/25
        
        public static final double FOLLOW_SPEED = 0.5; // Type: Meters
        public static final double BACKUP_DIST = .7; // Type: Meters

        public static final double DISTANCE_CONVERSION_FACTOR = 0.333; // 1/3

        public static final int TRACKED_TAG_ID = 0; 
    }

    public class Climber {
        public static final int CLIMBER_MOTOR_PORT = -1; // Type: kraken [FIXME]
    }

    public class Turret {
        public static final int SPITTER_MOTOR_PORT = 46; // Type: kraken [FIXME]
        public static final int HORIZONTAL_MOTOR_PORT = 25; // Type: kraken [FIXME]
        public static final int VERTICAL_MOTOR_PORT = 45; // Type: kraken [FIXME]

        public static final int SPINDEXER_MOTOR_PORT = 43; // Type: flex/vortex
        public static final int KICKER_MOTOR_PORT = 44; // Type: flex/vortex

        public class ShootConfig {
            public static final double SPINDEXER_SPEED = 0.2; // Units: percent (0-1)
            public static final double KICKER_SPEED = 0.8; // Units: percent (0-1)
            public static final double SPITTER_SPEED = 1; // Units: percent (0-1)
            public static final double INTAKE_SPEED = 0.3; // Units: percent (0-1)
            public static final double INTAKE_SPEED_RPM = 2000; // Unites: RPM (max is around 5700)

            public static final double KICKER_INIT_DELAY = 1; // Units: seconds
        }

        public class Horizontal {
            public static final double ANGLE_TURN_THRESHOLD = 2; // Units: degrees
            public static final double ANGLE_TURN_PERCENTAGE = 0.9; // Units: percent (0-1)
            public static final double MAX_TRACK_STEP_DEGREES = 4.0; // Units: degrees
            public static final double MAX_TRACK_RATE_DEGREES_PER_SEC = 120.0; // Units: degrees/seconds

            public static final double CONVERSION_FACTOR_MtoT = 185.0 / 28.0; // Motor rotations -> turret rotations
            public static final double CONVERSION_FACTOR_TtoM = 28.0 / 185.0; // Turret rotations -> motor rotations

            public static final double ANGLE_LIM_LEFT = -80.0; // Units: degrees
            public static final double ANGLE_LIM_RIGHT = 80.0; // Units: degrees
        }

        public class Vertical {
            public static final double CONVERSION_FACTOR_MtoH = 325.0 / 18.0; // Motor rotations -> hood rotations
            public static final double CONVERSION_FACTOR_HtoM = 18.0 / 325.0; // Hood rotations -> motor rotations

            public static final double ANGLE_LIM_LEFT = -31.0; // Units: degrees
            public static final double ANGLE_LIM_RIGHT = 0.0; // Units: degrees
        }

        public class Offset {
            public static final double X = 0.0; // Units: meters [FIXME]
            public static final double Y = 0.0; // Units: meters [FIXME]
            public static final double Z = 0.0; // Units: meters [FIXME]
        }
    }

    public class Intake {
        public static final int INTAKE_EXTENDER_MOTOR_PORT = 55; // Type: kraken
        public static final int MAIN_INTAKE_MOTOR_PORT = 57; // Type: kraken

        public static final double CONVERSION_FACTOR_MtoE = 14 / 64; // Motor rotations -> extender rotations
        public static final double CONVERSION_FACTOR_EtoM = 64 / 14; // extender rotations -> Motor rotations
    }

    public class RelativeHubLocation {
        // Assuming +x is away from driver station, and +y is to the right
        // Units: in.
        public static final Map<Integer, Translation2d> BLUE_APRIL_POS = Map.of(
            18, new Translation2d(0.0, 23.5),
            27, new Translation2d(-17.5, 23.5),
            26, new Translation2d(-23.5, 0.0),
            25, new Translation2d(-23.5, -17.5),
            24, new Translation2d(-17.5, -23.5),
            21, new Translation2d(0.0, -23.5),
            19, new Translation2d(23.5, 0.0),
            20, new Translation2d(23.5, -17.5)
        );

        // Units: in.
        public static final Map<Integer, Translation2d> RED_APRIL_POS = Map.of(
            5, new Translation2d(0.0, 23.5),
            8, new Translation2d(-17.5, 23.5),
            10, new Translation2d(-23.5, 17.5),
            9, new Translation2d(-23.5, 0.0),
            11, new Translation2d(-17.5, -23.5),
            2, new Translation2d(0.0, -23.5),
            3, new Translation2d(23.5, 17.5),
            4, new Translation2d(23.5, 0.0)
        );

        // [TODO]: make init for it use Elastic dashboard instead of constant
        public static final Map<Integer, Translation2d> MY_APRIL_POS =
            "BLUE".equals(TEAM) ? BLUE_APRIL_POS : RED_APRIL_POS;

        public static final double Z_POS = 48.6; // Units: in.
        public static final double CENTER_OFFSET_MARGIN = 20.85; // Units: in.
    }
}
