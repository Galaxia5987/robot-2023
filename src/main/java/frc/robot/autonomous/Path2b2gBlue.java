package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;

/**
 * This class contains al the parts for the path 2nb2g.
 *
 * In this path the robot places a cube in the middle grid
 * in the upper part and goes to the charge station.
 */
public class Path2b2gBlue extends SequentialCommandGroup {
    public Path2b2gBlue() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/2nb2g blue"),
                new BalanceOnStation()
        );
    }
}
