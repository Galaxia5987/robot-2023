package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.UpperScoring;

/**
 * This class contains all the parts of the path LeftConeHighRun.
 *
 * In this path the robot places a cone in the grid that is closest to the feeder
 * and goes to park in the opposite alliance's loading zone.
 */
public class LeftConeHighRun extends SequentialCommandGroup {
    public LeftConeHighRun() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/LeftConeHighRun blue")
        );
    }
}
