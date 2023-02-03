package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.PickUpCube;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.ConstantsIntake;

/**
 * This class contains all the parts to the path 3n32b3b2.
 *
 * In this path the robot places a cone in the grid that is the furthest away from the feeder,
 * goes to pick up a cube (the one that's furthest from the feeder),
 * returns to place it in the same grid,
 * goes to pick up a second cube (the one next to the last one)
 * and goes to place it in the middle grid.
 */
public class Path3n32b3b2blue extends SequentialCommandGroup {
    public Path3n32b3b2blue() {
        addCommands(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 1"),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 2"),
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 3"),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 4"),
                new UpperScoring()
        );
    }
}
