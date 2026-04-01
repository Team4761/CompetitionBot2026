package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.Map;
import java.util.function.Function;

public final class Constants {
    private Constants() {}

    public static final class Robot {
        public static final double ROBOT_WIDTH = Units.inchesToMeters(27.0);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(27.0);
        public static final double ROBOT_HEIGHT_WITHOUT_TURRET = Units.inchesToMeters(19.375);

        public static final Function<Double, Double> ROBOT_HEIGHT_WITH_TURRET =
            theta -> ROBOT_HEIGHT_WITHOUT_TURRET
                + Units.inchesToMeters(Math.sin(Math.toRadians(theta)) * 4.5);

        private Robot() {}
    }

    public static final class Controller {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final double TRANSLATION_INPUT_DEADBAND = 0.10;
        public static final double ROTATION_INPUT_DEADBAND = 0.12;
        public static final double TURRET_INPUT_DEADBAND = 0.10;
        public static final double ROTATION_SLEW_RATE_RAD_PER_SEC_SQ = 12.0;
        public static final double ROTATION_MULTIPLIER = 2.0;

        public static final double TEST_VORTEX_OUTPUT = 0.20;
        public static final double TEST_KRAKEN_OUTPUT = 0.20;

        private Controller() {}
    }

    public static final class Swerve {
        public static final double MAX_DRIVE_SPEED = 0.5;

        private Swerve() {}

        public static final class Auto {
            public static final double TRANSLATION_KP = 2.5;
            public static final double ROTATION_KP = 4.0;

            public static final double MAX_TRANSLATION_SPEED_MPS = 2.0;
            public static final double MAX_ROTATION_SPEED_RAD_PER_SEC =
                Units.degreesToRadians(180.0);

            public static final double POSITION_TOLERANCE_METERS = 0.03;
            public static final double ANGLE_TOLERANCE_DEGREES = 2.0;

            private Auto() {}
        }
    }

    public static final class Gyro {
        public static final int PIGEON_ID = 0;

        private Gyro() {}
    }

    public static final class Vision {
        public static final String LEFT_CAMERA_NAME = "Back Left Cam";
        public static final String RIGHT_CAMERA_NAME = "Back Right Cam";

        public static final Translation3d LEFT_CAM_TRANSLATION =
            new Translation3d(
                Units.inchesToMeters(-12.0),
                Units.inchesToMeters(12.0),
                Units.inchesToMeters(17.5)
            );
        public static final Translation3d RIGHT_CAM_TRANSLATION =
            new Translation3d(
                Units.inchesToMeters(-12.0),
                Units.inchesToMeters(-12.0),
                Units.inchesToMeters(17.5)
            );

        public static final Rotation3d LEFT_CAM_ROTATION =
            new Rotation3d(0.0, 0.0, Math.toRadians(135.0));
        public static final Rotation3d RIGHT_CAM_ROTATION =
            new Rotation3d(0.0, 0.0, Math.toRadians(215.0));

        public static final Transform3d LEFT_CAM_TRANSFORM =
            new Transform3d(LEFT_CAM_TRANSLATION, LEFT_CAM_ROTATION);
        public static final Transform3d RIGHT_CAM_TRANSFORM =
            new Transform3d(RIGHT_CAM_TRANSLATION, RIGHT_CAM_ROTATION);

        public static final double ANGLE_DEADBAND = 2.0;

        private Vision() {}
    }

    public static final class Turret {
        public static final double MAX_SPEED_MEASURED_MpS = 13.5;

        public static final int SPITTER_MOTOR_PORT = 46;
        public static final int KICKER_MOTOR_PORT = 44;
        public static final int HORIZONTAL_MOTOR_PORT = 25;
        public static final int VERTICAL_MOTOR_PORT = 45;
        public static final int SPINDEXER_MOTOR_PORT = 43;

        private Turret() {}

        public static final class ShootConfig {
            public static final double SPINDEXER_SPEED = 0.8;
            public static final double KICKER_SPEED = 0.8;

            public static final double SPITTER_SPEED = 7000.0;
            public static final double MED_SPITTER_SPEED = 5500.0;
            public static final double SHORT_SPITTER_SPEED = 4300.0;
            public static final double AUTO_SPITTER_SPEED = 4800.0;

            public static final double INTAKE_SPEED = 0.55;
            public static final double INTAKE_SPEED_RPM = 20.0;

            public static final double KICKER_INIT_DELAY = 0.25;
            public static final double KICKER_MOTOR_ROTATIONS_PER_ROTATION = 3.0;

            public static final double TURRET_HORIZONTAL_MULTIPLIER = 3.25;
            public static final double TURRET_VERTICAL_MULTIPLIER = 1.95;

            private ShootConfig() {}
        }

        public static final class Horizontal {
            public static final double MAX_TRACK_STEP_DEGREES = 4.0;

            public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATION = 185.0 / 28.0;
            public static final double TURRET_ROTATIONS_PER_MOTOR_ROTATION = 28.0 / 185.0;

            public static final double ANGLE_LIM_LEFT = -80.0;
            public static final double ANGLE_LIM_RIGHT = 80.0;

            private Horizontal() {}
        }

        public static final class Vertical {
            public static final double MOTOR_ROTATIONS_PER_HOOD_ROTATION = 325.0 / 18.0;
            public static final double HOOD_ROTATIONS_PER_MOTOR_ROTATION = 18.0 / 325.0;

            public static final double MIN_HOOD_ANGLE_DEGREES = -31.0;
            public static final double MAX_HOOD_ANGLE_DEGREES = 0.0;

            public static final double MIN_LAUNCH_ANGLE_DEGREES = 22.0;
            public static final double MAX_LAUNCH_ANGLE_DEGREES =
                MIN_LAUNCH_ANGLE_DEGREES + MAX_HOOD_ANGLE_DEGREES;

            private Vertical() {}
        }

        public static final class Offset {
            public static final double X = 0.0;
            public static final double Y = -6.5;

            private Offset() {}
        }
    }

    public static final class Intake {
        public static final int INTAKE_EXTENDER_MOTOR_PORT = 55;
        public static final int MAIN_INTAKE_MOTOR_PORT = 57;

        public static final double MOTOR_ROTATIONS_PER_EXTENDER_ROTATION = 13.1875;
        public static final double EXTENDER_ROTATIONS_PER_MOTOR_ROTATION = 1.0 / 13.1875;

        public static final double MIN_EXTENSION_ANGLE = -105.0;
        public static final double MAX_EXTENSION_ANGLE = 0.0;

        public static final double RETRACTION_SPEED = 1.0;
        public static final double SECONDS_IT_TAKES_TO_RETRACT_AT_FULL_SPEED = 5.0;

        private Intake() {}
    }

    public class Field {
        public static String STARTING_POSITION = "CENTER"; // Type: enum("LEFT", "CENTER", "RIGHT")

    }
}
