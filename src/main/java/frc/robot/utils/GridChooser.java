package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GridChooser {
    private final XboxController xboxController;

    private int index = 0;
    private final boolean[] grid = new boolean[9];

    public GridChooser(XboxController xboxController) {
        this.xboxController = xboxController;
        for (int i = 0; i < 9; i++) {
            grid[i] = i == index;
        }
    }

    public void update() {
        int pov = xboxController.getPOV();

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
            SmartDashboard.putBoolean("Grid #" + (i + 1), grid[i]);
        }
    }
}
