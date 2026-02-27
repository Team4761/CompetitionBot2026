package frc.robot;

import org.javatuples.Pair;
import java.util.HashMap;
import java.util.Map;

public class Constants {
    // Controller
    public static final int CONTROLLER_PORT = 0; // Port on the Driverstation
    // Port for the weapons/operator controller
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final String TEAM = "BLUE"; // Type: enum("BLUE", "RED")

    public class Swerve {
        public static final double MAX_DRIVE_SPEED = 0.5; // Meters per second

        // Unneeded, but here for documentation
        public class FrontLeft {
            public static final int TURN_MOTOR_PORT = -1;
            public static final int DRIVE_MOTOR_PORT = -1;
        }
        public class FrontRight {
            public static final int TURN_MOTOR_PORT = -1;
            public static final int DRIVE_MOTOR_PORT = -1;
        }
        public class BackLeft {
            public static final int TURN_MOTOR_PORT = -1;
            public static final int DRIVE_MOTOR_PORT = -1;
        }
        public class BackRight {
            public static final int TURN_MOTOR_PORT = 11;
            public static final int DRIVE_MOTOR_PORT = -1;
        }
    }

    public class Gyro {
        public static final int PIGEON_ID = 0; // Type: pigeon2 [FIXME]
    }

    public class Vision {
        public static final int TRACKED_TAG_ID = 7;
    }

    public class Turret {
        public static final int SPINDEXER_MOTOR_PORT = 15;
        public static final int KICKER_MOTOR_PORT = 16;
        public static final int SPITTER_MOTOR_PORT = 17;
        public static final int HORIZONTAL_MOTOR_PORT = 18;
        public static final int VERTICAL_MOTOR_PORT = 19;

        // NEW: Default speed for the kicker feeding the shooter
        public static final double KICKER_SPEED = 0.5;
        public static final double SPITTER_SPEED = 0.2; // Units - percent (0-1)

        public class Vertical {
            public static final double ANGLE_LIM_LEFT = -45.0;
            public static final double ANGLE_LIM_RIGHT = 45.0;
            public static final double CONVERSION_FACTOR_HtoM = 1.0;
        }
    }

    public class Intake {
        public static final int INTAKE_EXTENDER_MOTOR_PORT = 20;
        public static final int MAIN_INTAKE_MOTOR_PORT = 21;
    }

    public class RelativeHubLocation {
        // Assuming +x is away from driver station, and +y is to the right
        // Units: in.
        public static final Map<Integer, Pair<Double, Double>> BLUE_APRIL_POS = Map.of(
            18, new Pair<>(0.0, 23.5),
            27, new Pair<>(-17.5, 23.5),
            26, new Pair<>(-23.5, 0.0),
            25, new Pair<>(-23.5, -17.5),
            24, new Pair<>(-17.5, -23.5),
            21, new Pair<>(0.0, -23.5),
            19, new Pair<>(23.5, 0.0),
            20, new Pair<>(23.5, -17.5)
        );

        // Units: in.
        public static final Map<Integer, Pair<Double, Double>> RED_APRIL_POS = Map.of(
            5, new Pair<>(0.0, 23.5),
            8, new Pair<>(-17.5, 23.5),
            10, new Pair<>(-23.5, 17.5),
            9, new Pair<>(-23.5, 0.0),
            11, new Pair<>(-17.5, -23.5),
            2, new Pair<>(0.0, -23.5),
            3, new Pair<>(23.5, 17.5),
            4, new Pair<>(23.5, 0.0)
        );

        // [TODO]: make init for it use Elastic dashboard instead of constant
        public static final Map<Integer, Pair<Double, Double>> MY_APRIL_POS = (TEAM == "BLUE") ? BLUE_APRIL_POS : RED_APRIL_POS;

        public static final double Z_POS = 48.6; // Units: in.
        public static final double CENTER_OFFSET_MARGIN = 20.85; // Units: in.
    }
}
