package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GridChooser {
    private final boolean[] grid = new boolean[9];
    private int index = 0;

    public GridChooser() {
        for (int i = 0; i < 9; i++) {
            grid[i] = i == index;
        }
    }

    public void update(int pov) {
        if (Utils.epsilonEquals(pov, 90, 50)) {
            if (index == 8) {
                index = 0;
            } else {
                index++;
            }
        } else if (Utils.epsilonEquals(pov, 270, 50)) {
            if (index == 0) {
                index = 8;
            } else {
                index--;
            }
        }

        for (int i = 0; i < 9; i++) {
            grid[i] = i == index;
        }

        for (int i = 0; i < 9; i++) {
            SmartDashboard.putBoolean("Grid #" + (i + 1), grid[i]);
        }
    }

    public int getAprilTagIndex() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            if (index / 3 == 0) {
                return 6;
            } else if (index / 3 == 1) {
                return 7;
            } else {
                return 8;
            }
        } else {
            if (index / 3 == 0) {
                return 1;
            } else if (index / 3 == 1) {
                return 2;
            } else {
                return 3;
            }
        }
    }

    public int getIndex() {
        return index;
    }

    public Position getPosition() {
        return new Position(index + 1, getAprilTagIndex());
    }

    public static class Position {
        public int index;
        public int aprilTagID;

        public Position(int index, int aprilTagID) {
            this.index = index;
            this.aprilTagID = aprilTagID;
        }
    }
}
