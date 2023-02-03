package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.UpperScoring;

/**
 * This class contains all the parts of the path 1n11r.
 *
 * In this path the robot places a cone in the grid that is closest to the feeder
 * and goes to park in the opposite alliance's loading zone.
 */
public class Path1n11rblue extends SequentialCommandGroup {
    public Path1n11rblue() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11r blue")
        );
    }
}
