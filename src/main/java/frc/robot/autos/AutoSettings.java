package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.FieldConstants;

public record AutoSettings(Alliance alliance, StartingPosition startingPosition) {
    private static final double LEFT_LANE_OFFSET_METERS =
        (FieldConstants.Field.FIELD_WIDTH / 2.0) - FieldConstants.Depot.DEPOT_DISTANCE_FROM_LEFT_EDGE;
    private static final double RIGHT_LANE_OFFSET_METERS =
        -((FieldConstants.Field.FIELD_WIDTH / 2.0) - FieldConstants.Outpost.OUTPOST_DISTANCE_FROM_RIGHT_EDGE);

    public static AutoSettings fromDashboardSelections() {
        return new AutoSettings(
            Alliance.fromString(Constants.Field.ALLIANCE_COLOR),
            StartingPosition.fromString(Constants.Field.STARTING_POSITION)
        );
    }

    public double shiftMetersTo(StartingPosition targetPosition) {
        return targetPosition.centerlineOffsetMeters() - startingPosition.centerlineOffsetMeters();
    }

    public enum Alliance {
        RED,
        BLUE;

        public static Alliance fromString(String value) {
            return "RED".equalsIgnoreCase(value) ? RED : BLUE;
        }
    }

    public enum StartingPosition {
        // These three dashboard positions are treated as driver-station-relative lane centers.
        // LEFT is aligned to the DEPOT side, CENTER is aligned to the HUB/TRENCH center lane,
        // and RIGHT is aligned to the OUTPOST side.
        LEFT,
        CENTER,
        RIGHT;

        public double centerlineOffsetMeters() {
            return switch (this) {
                case LEFT -> LEFT_LANE_OFFSET_METERS;
                case CENTER -> 0.0;
                case RIGHT -> RIGHT_LANE_OFFSET_METERS;
            };
        }

        public static StartingPosition fromString(String value) {
            if ("LEFT".equalsIgnoreCase(value)) {
                return LEFT;
            }
            if ("RIGHT".equalsIgnoreCase(value)) {
                return RIGHT;
            }
            return CENTER;
        }
    }
}
