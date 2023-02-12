package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.vision.Limelight;

/**
 * This class contains all the parts of the path LeftConeHighRun.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder
 * and goes to park in the opposite alliance's loading zone.
 */
public class LeftConeHighRun extends SequentialCommandGroup {

    public LeftConeHighRun() {
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setTapePipeline, limelight),
                new UpperScoring(),
                FollowPath.loadTrajectory("pathplanner/LeftConeHighRun blue")
        );
    }
}
