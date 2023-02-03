package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;

/**
 * This class contains al the parts for the path MiddleConeHighEngage.
 *
 * In this path the robot places a cone in the middle grid
 * in the part that is closer to the feeder and goes to the charge station.
 */
public class MiddleConeHighEngage extends SequentialCommandGroup {
    public MiddleConeHighEngage() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/MiddleConeHighEngage blue"),
                new BalanceOnStation()
        );
    }
}
