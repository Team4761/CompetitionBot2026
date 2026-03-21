package frc.robot;
import edu.wpi.first.math.util.Units;

// All Constants below have been taken from https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf and the REBUILT Game Manual
// Since we're attending the NE FIRST Competition, we use AndyMark field dimensions
// The field dimensions in the manual are based off of Welded, but we don't get welded. yippee (basically applies to field only)
// All Units have been converted from inches to meters for your convenience 
public class FieldConstants {
    // Field dimensions
    /**
     * Describes all general field dimensions, including the field itself, the alliance zones, and the neutral zone.
     */

    public class Field {
        public static final double FIELD_WIDTH = Units.inchesToMeters(316.64); // Units: meters 
        public static final double FIELD_LENGTH = Units.inchesToMeters(650.12); // Units: meters

        public static final double ALLIANCE_ZONE_WIDTH = Units.inchesToMeters(316.64); // Units: meters
        public static final double ALLIANCE_ZONE_LENGTH = Units.inchesToMeters(158.6); // Units: meters

        public static final double NEUTRAL_ZONE_WIDTH = Units.inchesToMeters(316.64); // Units: meters
        public static final double NEUTRAL_ZONE_LENGTH = Units.inchesToMeters(287.0); // Units: meters
    }

    public class Fuel {
        public static final double FUEL_DIAMETER = Units.inchesToMeters(5.906); // Units: meters
        public static final double FUEL_BOUND_BOX_WIDTH = Units.inchesToMeters(181.90); // Units: meters
        public static final double FUEL_BOUND_BOX_LENGTH = Units.inchesToMeters(71.90); // Units: meters
    }

    public class Bump {
        public static final double BUMP_WIDTH = Units.inchesToMeters(44.4); // Units: meters
        public static final double BUMP_LENGTH = Units.inchesToMeters(73.0); // Units: meters
        public static final double BUMP_HEIGHT = Units.inchesToMeters(6.513); // Units: meters
        public static final double BUMP_ELEVATION = Units.inchesToMeters(0.61); // Units: meters (from dimension pdf)
        public static final double BUMP_DISTANCE_FROM_ALLIANCE_WALL = Units.inchesToMeters(158.06); // Units: meters (AndyMark)
    }

    public class Trench {
        public static final double TRENCH_WIDTH = Units.inchesToMeters(47.0); // Units: meters
        public static final double TRENCH_LENGTH = Units.inchesToMeters(65.65); // Units: meters
        public static final double TRENCH_HEIGHT = Units.inchesToMeters(40.25); // Units: meters (this one is NOT the space underneath the trenches)
        public static final double TRENCH_OPENING_LENGTH = Units.inchesToMeters(50.34); // Units: meters
        public static final double TRENCH_OPENING_HEIGHT = Units.inchesToMeters(22.25); // Units: meters
        public static final double TRENCH_DISTANCE_FROM_ALLIANCE_WALL = Units.inchesToMeters(158.06); // Units: meters (AndyMark)
    }
    
    public class Depot {
        public static final double DEPOT_DISTANCE_FROM_LEFT_EDGE = Units.inchesToMeters(82.32); // Units: meters (AndyMark) (from left edge of field to center of depot)
        public static final double DEPOT_WIDTH = Units.inchesToMeters(27.0); // Units: meters
        public static final double DEPOT_LENGTH = Units.inchesToMeters(42.0); // Units: meters
        public static final double DEPOT_BORDER_WIDTH = Units.inchesToMeters(3.0); // Units: meters
        public static final double DEPOT_BORDER_HEIGHT = Units.inchesToMeters(1.125); // Units: meters (the actual height of the border is 1 inch, but hook fasteners add an additional eighth, according to the manual)
    }   

    public class Tower {
        public static final double TOWER_WIDTH = Units.inchesToMeters(45.0); // Units: meters
        public static final double TOWER_LENGTH = Units.inchesToMeters(49.25); // Units: meters
        public static final double TOWER_HEIGHT = Units.inchesToMeters(78.25); // Units: meters

        public static final double UPRIGHTS_DISTANCE_FROM_WALL = Units.inchesToMeters(40.0); // Units: meters (AndyMark)

        public static final double RUNG_DIAMETER = Units.inchesToMeters(1.66); // Units: meters
        public static final double RUNG_LENGTH = Units.inchesToMeters(32.250); // Units: meters

        public static final double L1_HEIGHT = Units.inchesToMeters(27.0); // Units: meters (From floor to center of rung)
        public static final double L2_HEIGHT = Units.inchesToMeters(45.0); // Units: meters (From floor to center of rung)
        public static final double L3_HEIGHT = Units.inchesToMeters(63.0); // Units: meters (From floor to center of rung)
    }

    public class Hub {
        public static final double HUB_TO_ALLIANCE_WALL = Units.inchesToMeters(158.06); // Units: meters (AndyMark)
        public static final double HUB_SIZE = Units.inchesToMeters(47.0); // Units: meters (length & width)
        public static final double HUB_HEIGHT = Units.inchesToMeters(72); // Units: meters
        public static final double HUB_OPENING_WIDTH = Units.inchesToMeters(41.7); // Units: meters
        //public static final double HUB_BLUE_TO_BOTTOM_LEFT_CORNER = (Math.hypot(HUB_TO_ALLIANCE_WALL, Field.FIELD_WIDTH/2)); //distance from blue feild hub to bottome left corner on elastic deaboard map
        //public static final double HUB_RED_TO_BOTTOM_LEFT_CORNER = (Math.hypot((Field.FIELD_LENGTH - HUB_TO_ALLIANCE_WALL), Field.FIELD_WIDTH/2)); // distance from reed hub to bottm left corner of elastic dashboard map
        public static final double HUB_BLUE_X = HUB_TO_ALLIANCE_WALL;//x pos of blue hub compared to bottom left corner of elastic dashboard map
        public static final double HUB_BLUE_Y = Field.FIELD_WIDTH/2;//y pos of blue hub compared to bottom left corner of elastic dashboard map
        public static final double HUB_RED_X = Field.FIELD_LENGTH - HUB_TO_ALLIANCE_WALL;//x pos of red hub compared to bottom left corner of elastic dashboard map
        public static final double HUB_RED_Y = Field.FIELD_WIDTH/2;//y pos of red hub compared to bottom left corner of elastic dashboard map

    }

    public class Outpost {
        public static final double CHUTE_OPENING_WIDTH = Units.inchesToMeters(31.8); // Units: meters
        public static final double CHUTE_OPENING_HEIGHT = Units.inchesToMeters(7.0); // Units: meters
        public static final double CHUTE_OPENING_ELEVATION = Units.inchesToMeters(28.1); // Units: meters (from floor to bottom of opening)
        
        public static final double CORRAL_OPENING_WIDTH = Units.inchesToMeters(32.0); // Units: meters
        public static final double CORRAL_OPENING_HEIGHT = Units.inchesToMeters(7.0); // Units: meters
        public static final double CORRAL_OPENING_ELEVATION = Units.inchesToMeters(1.88);

        public static final double OUTPOST_DISTANCE_FROM_RIGHT_EDGE = Units.inchesToMeters(25.62); // Units: meters (AndyMark) (from right edge of field to center of outpost)
    }

    public class Match {
        public static final double MATCH_DURATION = 165; // Units: seconds
        public static final double AUTONOMOUS_DURATION = 20; // Units: seconds
        public static final double TRANSITION_DURATION = 15; // Units: seconds
        public static final double ALLIANCE_SHIFTS = 4; // amount of unit shifts
        public static final double ALLIANCE_SHIFT_DURATION = 25; // Units: seconds (how long each alliance shift lasts before switching)
        public static final double ENDGAME_DURATION = 30; // Units: seconds (the last 30 seconds of the match)
    }

    public class AprilTag {
        public static final double TAG_SIZE = Units.inchesToMeters(6.5); // Units: meters
        public static final double TAG_BOARD_SIZE = Units.inchesToMeters(10.5); // Units: meters

        public static final double APRILTAG_HUB_HEIGHT = Units.inchesToMeters(44.25); // Units: meters (from floor to center of tag)
        public static final double APRILTAG_TOWER_HEIGHT = Units.inchesToMeters(21.75); // Units: meters (from floor to center of tag
        public static final double APRILTAG_OUTPOST_HEIGHT = Units.inchesToMeters(21.75); // Units: meters (from floor to center of tag)
        public static final double APRILTAG_TRENCH_HEIGHT = Units.inchesToMeters(35); // Units: meters (from floor to center of tag)
    }

}
