package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.command_groups.PickUpCube;
import frc.robot.command_groups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.ConstantsIntake;

/**
 * This class contains all the parts to the path RightConeCubeHigh.
 *
 * In this path the robot places a cone in the grid that is the furthest away from the feeder,
 * goes to pick up a cube (the one furthest from the feeder)
 * and returns to place it in the same grid.
 */
public class RightConeCubeHigh extends SequentialCommandGroup {
    public RightConeCubeHigh() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHigh blue 1"),
                new PickUpCube(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHigh blue 2"),
                new UpperScoring()
        );
    }
}
