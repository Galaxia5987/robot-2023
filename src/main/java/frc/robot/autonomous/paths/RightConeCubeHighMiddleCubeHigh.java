package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.command_groups.PickUpCube;
import frc.robot.command_groups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.ConstantsIntake;

/**
 * This class contains all the parts to the path RightConeCubeHighMiddleCubeHigh.
 *
 * In this path the robot places a cone in the grid that is the furthest away from the feeder,
 * goes to pick up a cube (the one that's furthest from the feeder),
 * returns to place it in the same grid,
 * goes to pick up a second cube (the one next to the last one)
 * and goes to place it in the middle grid.
 */
public class RightConeCubeHighMiddleCubeHigh extends SequentialCommandGroup {
    public RightConeCubeHighMiddleCubeHigh() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHighMiddleCubeHigh blue 1"),
                new PickUpCube(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHighMiddleCubeHigh blue 2"),
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHighMiddleCubeHigh blue 3"),
                new PickUpCube(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHighMiddleCubeHigh blue 4"),
                new UpperScoring()
        );
    }
}
