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
 * This class contains all the parts to the path 3n32b3b2.
 *
 * In this path the robot places a cone in the grid that is the furthest away from the feeder,
 * goes to pick up a cube (the one that's furthest from the feeder),
 * returns to place it in the same grid,
 * goes to pick up a second cube (the one next to the last one)
 * and goes to place it in the middle grid.
 */
public class Path3n32b3b2blue {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public CommandBase placeUpperCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    public CommandBase placeMiddleCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    public CommandBase followPathAndPlaceUppercube3(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 2", swerveDrive),
                new UpperScoring()
        );
    }

    public CommandBase followPathAndPlaceMiddlecube3(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 2", swerveDrive),
                new MiddleScoring()
        );
    }

    public CommandBase followPathAndPickUpCube(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 3", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    public CommandBase followPathAndPlaceUpperCube2(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 4", swerveDrive),
                new UpperScoring()
        );
    }

    public CommandBase followPathAndPlaceMiddleCube2(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 4", swerveDrive),
                new MiddleScoring()
        );
    }
}
