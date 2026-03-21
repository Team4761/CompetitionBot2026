package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.FieldConstants;

public record AutoSettings(Alliance alliance, StartingPosition startingPosition) {
    public static final double LANE_SPACING_METERS = FieldConstants.Field.ALLIANCE_ZONE_WIDTH / 4.0;

    public static AutoSettings fromDashboardSelections() {
        return new AutoSettings(
            Alliance.fromString(Constants.Field.ALLIANCE_COLOR),
            StartingPosition.fromString(Constants.Field.STARTING_POSITION)
        );
    }

    public double shiftMetersTo(StartingPosition targetPosition) {
        int deltaLanes = targetPosition.laneIndex() - startingPosition.laneIndex();
        return -deltaLanes * LANE_SPACING_METERS;
    }

    public double trenchEntryTurnDegrees() {
        return switch (startingPosition) {
            case LEFT -> -35.0;
            case CENTER -> 0.0;
            case RIGHT -> 35.0;
        };
    }

    public enum Alliance {
        RED,
        BLUE;

        public static Alliance fromString(String value) {
            return "RED".equalsIgnoreCase(value) ? RED : BLUE;
        }
    }

    public enum StartingPosition {
        LEFT(-1),
        CENTER(0),
        RIGHT(1);

        private final int laneIndex;

        StartingPosition(int laneIndex) {
            this.laneIndex = laneIndex;
        }

        public int laneIndex() {
            return laneIndex;
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
