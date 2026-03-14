package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.Map;
import java.util.function.Function;

public class Constants {
    // Controller
    public static final int CONTROLLER_PORT = 0; // Port on the Driverstation

    public class Robot {
        public static final double ROBOT_WIDTH = Units.inchesToMeters(27.0); // Units: meters
        public static final double ROBOT_LENGTH = Units.inchesToMeters(27.0); // Units: meters
        public static final double ROBOT_HEIGHT_WITHOUT_TURRET = Units.inchesToMeters(19.375); // Units: meters

        public static final Function<Double, Double> ROBOT_HEIGHT_WITH_TURRET = (theta) -> ROBOT_HEIGHT_WITHOUT_TURRET + Units.inchesToMeters(Math.sin(Math.toRadians(theta)) * 4.5); // Units: meters
    }

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

        public class Auto {
            public static final double TRANSLATION_KP = 2.5;
            public static final double ROTATION_KP = 4.0;

            public static final double MAX_TRANSLATION_SPEED_MPS = 2.0; // Units: meters per second
            public static final double MAX_ROTATION_SPEED_RAD_PER_SEC = Units.degreesToRadians(180.0);

            public static final double POSITION_TOLERANCE_METERS = 0.03;
            public static final double ANGLE_TOLERANCE_DEGREES = 2.0;
        }
    }

    public class Gyro {
        public static final int PIGEON_ID = 0; // Type: pigeon2 
    }

    public class Vision {
        public static final String DEFAULT_CAMERA_NAME = "goodCam"; // Type: string

        public static final double ANGLE_DEADBAND = 2.0; // Units: degrees
        public static final double CAMERA_OFFSET_FROM_TURRET_X = Units.inchesToMeters(9.5); // Units: meters
        public static final double CAMERA_OFFSET_FROM_TURRET_Y = Units.inchesToMeters(0.0); // Units: meters
        public static final Function<Double, Double> CAMERA_OFFSET_FROM_TURRET_Z = (theta) -> Units.inchesToMeters(Math.sin(Math.toRadians(theta)) * 4.5); // Units: meters
    }

    public class Climber {
        public static final int CLIMBER_MOTOR_PORT = -1; // Type: kraken [FIXME]
    }

    public class Turret {
        public static final double MAX_SPEED_MEASURED_MpS = 13.5; // Units: m/s
        public static final double MAX_RPM = 5700.0; // Units: RPM

        public static final int SPITTER_MOTOR_PORT = 46; // Type: kraken 
        public static final int HORIZONTAL_MOTOR_PORT = 25; // Type: kraken 
        public static final int VERTICAL_MOTOR_PORT = 45; // Type: kraken 

        public static final int SPINDEXER_MOTOR_PORT = 43; // Type: flex/vortex
        public static final int KICKER_MOTOR_PORT = 44; // Type: flex/vortex

        public class ShootConfig {
            public static final double SPINDEXER_SPEED = 0.4; // Units: percent (0-1)

            public static final double KICKER_SPEED = 0.4; // Units: percent (0-1)
            public static final double SPITTER_SPEED = 1; // Units: percent (0-1)
            public static final double INTAKE_SPEED = 0.4; // Units: percent (0-1)
            public static final double INTAKE_SPEED_RPM = 20; // Unites: RPM (max is around 5700)

            public static final double KICKER_INIT_DELAY = 1; // Units: seconds

            public static final double TURRET_HORIZONTAL_MULTIPLIER = 3.25;
            public static final double TURRET_VERTICAL_MULTIPLIER = 1.95;
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
            public static final double CONVERSION_FACTOR_HtoM = 325.0 / 18.0; // Hood rotations -> motor rotations
            public static final double CONVERSION_FACTOR_MtoH = 18.0 / 325.0; // Motor rotations -> hood rotations

            public static final double MIN_HOOD_ANGLE_DEGREES = -31.0; // Units: degrees from bottom stop
            public static final double MAX_HOOD_ANGLE_DEGREES = 0.0; // Units: safe degrees from bottom stop

            public static final double MIN_LAUNCH_ANGLE_DEGREES = 22.0; // Units: degrees from horizontal at bottom stop
            public static final double MAX_LAUNCH_ANGLE_DEGREES =
                MIN_LAUNCH_ANGLE_DEGREES + MAX_HOOD_ANGLE_DEGREES; // Units: degrees from horizontal
        }

        public class Offset {
            public static final double X = 0.0; // Units: meters
            public static final double Y = -6.5; // Units: meters
        }
    }

    public class Intake {
        public static final int INTAKE_EXTENDER_MOTOR_PORT = 55; // Type: kraken
        public static final int MAIN_INTAKE_MOTOR_PORT = 57; // Type: kraken

        public static final double CONVERSION_FACTOR_MtoE = 13.1875; // Motor rotations -> extender rotations
        public static final double CONVERSION_FACTOR_EtoM = 1 / 13.1875; // extender rotations -> Motor rotations

        public static final double MIN_EXTENSION_ANGLE = -105.0; // Units: degrees
        public static final double MAX_EXTENSION_ANGLE = 0.0; // Units: degrees

        public static final double RETRACTION_SPEED = 1.0; // Units: percent (0-1)

        public static final double SECONDS_IT_TAKES_TO_RETRACT_AT_FULL_SPEED = 5.0; // temp value [FIXME]
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

        // [FIXME]: make init for it use Elastic dashboard instead of constant
        public static final Map<Integer, Translation2d> MY_APRIL_POS =
            "BLUE".equals(Field.ALLIANCE_COLOR) ? BLUE_APRIL_POS : RED_APRIL_POS;

        public static final double Z_POS = 48.6; // Units: in.
        public static final double CENTER_OFFSET_MARGIN = 20.85; // Units: in.
    }
    public class Field {
        public static String ALLIANCE_COLOR = "BLUE"; // Type: enum("BLUE", "RED")
        public static String STARTING_POSITION = "CENTER"; // Type: enum("LEFT", "CENTER", "RIGHT")

    }
}
