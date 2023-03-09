package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.subsystems.vision.Limelight;

/**
 * This calss contains all the parts of the path LeftCubeHighCubeLow.
 * <p>
 * In this path the robot places a cube in the upper part of the grid that is closest to the feeder,
 * goes to get another cube (the one that is closest to the feeder)
 * and places it in the middle part of the same grid.
 */
public class LeftCubeHighCubeLow extends SequentialCommandGroup {

    public LeftCubeHighCubeLow() {
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setAprilTagsPipeline, limelight),
//                new UpperScoring(),
                FollowPath.loadTrajectory("pathplanner/LeftCubeHighCubeLow blue 1"),
                new PickUpCube(),
                FollowPath.loadTrajectory("pathplanner/LeftCubeHighCubeLow blue 2")
//                new MidScoring()
        );
    }
}
