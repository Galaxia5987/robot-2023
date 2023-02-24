package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.vision.Limelight;

/**
 * This class contains all the parts to the path RightConeCubeHighMiddleCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is the furthest away from the feeder,
 * goes to pick up a cube (the one that's furthest from the feeder),
 * returns to place it in the same grid,
 * goes to pick up a second cube (the one next to the last one)
 * and goes to place it in the middle grid.
 */
public class RightConeCubeHighMiddleCubeHigh extends SequentialCommandGroup {
    public RightConeCubeHighMiddleCubeHigh() {
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setTapeMiddlePipeline, limelight),
//                new UpperScoring(),
                FollowPath.loadTrajectory("pathplanner/RightConeCubeHighMiddleCubeHigh blue 1"),
                new InstantCommand(limelight::setAprilTagsPipeline, limelight),
                new PickUpCube(),
                FollowPath.loadTrajectory("pathplanner/RightConeCubeHighMiddleCubeHigh blue 2"),
//                new UpperScoring(),
                FollowPath.loadTrajectory("pathplanner/RightConeCubeHighMiddleCubeHigh blue 3"),
                new PickUpCube(),
                FollowPath.loadTrajectory("pathplanner/RightConeCubeHighMiddleCubeHigh blue 4")
//                new UpperScoring()
        );
    }
}
