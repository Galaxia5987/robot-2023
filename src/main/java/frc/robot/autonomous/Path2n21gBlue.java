package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;

/**
 * This class contains al the parts for the path 2n21g.
 *
 * In this path the robot places a cone in the middle grid
 * in the part that is closer to the feeder and goes to the charge station.
 */
public class Path2n21gBlue extends SequentialCommandGroup {
    public Path2n21gBlue() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/2n21g blue"),
                new BalanceOnStation()
        );
    }
}
