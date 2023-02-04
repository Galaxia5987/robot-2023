package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.command_groups.PickUpCube;
import frc.robot.command_groups.UpperScoring;

/**
 * This class contains all parts of the path LeftConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class LeftConeCubeHigh extends SequentialCommandGroup {
    public LeftConeCubeHigh() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/LeftConeCubeHigh blue 1"),
                new PickUpCube(),
                FollowPath.loadTrajectory(".pathplanner/LeftConeCubeHigh blue 2"),
                new UpperScoring()
        );
    }
}
