package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.MiddleScoring;
import frc.robot.commandGroups.PickUpCube;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.intake.ConstantsIntake;

/**
 * This calss contains all the parts of the path 1b1b1.
 *
 * In this pah the robot places a cube in the upper part of the grid that is closest to the feeder,
 * goes to get another cube (the one that is closest to the feeder)
 * and places it in the middle part of the same grid.
 */
public class Path1b1blue {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public CommandBase placeUpperCube1FollowPathAndPickUpCube(){
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/1b1b1 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    public CommandBase followPathAndPlaceMiddleCube1(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/1b1b1 blue 2", swerveDrive),
                new MiddleScoring()
        );
    }
}
