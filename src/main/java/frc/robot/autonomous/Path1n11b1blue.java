package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.PickUpCube;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.ConstantsIntake;

/**
 * This class contains all parts of the path 1n11b1.
 *
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class Path1n11b1blue extends SequentialCommandGroup {
    public Path1n11b1blue() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11b1 blue 1"),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE),
                FollowPath.loadTrajectory(".pathplanner/1n11b1 blue 2"),
                new UpperScoring()
        );
    }
}
