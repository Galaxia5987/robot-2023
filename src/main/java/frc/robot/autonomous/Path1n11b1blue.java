package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.MiddleScoring;
import frc.robot.commandGroups.PickUpCube;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.intake.ConstantsIntake;

public class Path1n11b1blue extends AutonomousPaths{
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    protected CommandBase placeUpperCone11FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11b1 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    protected CommandBase placeMiddleCone11FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11b1 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    protected CommandBase followPathAndPlaceUpperCube1() {
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/1n11b1 blue 2", swerveDrive),
                new UpperScoring()
        );
    }

    protected CommandBase followPathAndPlaceMiddleCube1() {
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/1n11b1 blue 2", swerveDrive),
                new MiddleScoring()
        );
    }
}