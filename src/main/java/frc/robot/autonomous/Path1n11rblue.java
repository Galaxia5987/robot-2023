package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.MiddleScoring;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * This class contains all the parts of the path 1n11r.
 *
 * In this path the robot places a cone in the grid that is closest to the feeder
 * and goes to park in the opposite alliance's loading zone.
 */
public class Path1n11rblue {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public CommandBase placeUpperCone11AndRun() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11r blue", swerveDrive)
        );
    }

    public CommandBase placeMiddleCone21AndRun() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11r blue", swerveDrive)
        );
    }
}
