package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.command_groups.UpperScoring;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;

/**
 * This class contains al the parts for the path MiddleCubeHighEngage.
 *
 * In this path the robot places a cube in the middle grid
 * in the upper part and goes to the charge station.
 */
public class MiddleCubeHighEngage extends SequentialCommandGroup {
    public MiddleCubeHighEngage() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/MiddleCubeHighEngage blue"),
                new BalanceOnStation()
        );
    }
}
