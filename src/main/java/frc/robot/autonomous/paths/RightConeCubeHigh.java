package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.vision.Limelight;

/**
 * This class contains all the parts to the path RightConeCubeHigh.
 *
 * In this path the robot places a cone in the grid that is the furthest away from the feeder,
 * goes to pick up a cube (the one furthest from the feeder)
 * and returns to place it in the same grid.
 */
public class RightConeCubeHigh extends SequentialCommandGroup {
    public RightConeCubeHigh() {
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setTapePipeline, limelight),
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHigh blue 1"),
                new InstantCommand(limelight::setAprilTagsPipeline, limelight),
                new PickUpCube(),
                FollowPath.loadTrajectory(".pathplanner/RightConeCubeHigh blue 2"),
                new UpperScoring()
        );
    }
}
