package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.command_groups.MidScoring;
import frc.robot.command_groups.PickUpCube;
import frc.robot.command_groups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.ConstantsIntake;

/**
 * This calss contains all the parts of the path LeftCubeHighCubeLow.
 *
 * In this pah the robot places a cube in the upper part of the grid that is closest to the feeder,
 * goes to get another cube (the one that is closest to the feeder)
 * and places it in the middle part of the same grid.
 */
public class LeftCubeHighCubeLow extends SequentialCommandGroup {

    public LeftCubeHighCubeLow() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/LeftCubeHighCubeLow blue 1"),
                new PickUpCube(),
                FollowPath.loadTrajectory(".pathplanner/LeftCubeHighCubeLow blue 2"),
                new MidScoring()
        );
    }
}
