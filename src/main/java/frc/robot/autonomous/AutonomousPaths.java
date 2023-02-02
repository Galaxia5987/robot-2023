package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.MiddleScoring;
import frc.robot.commandGroups.PickUpCube;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.ConstantsIntake;
import frc.robot.subsystems.intake.Intake;

public class AutonomousPaths {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    protected CommandBase placeUpperCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    protected CommandBase placeMiddleCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    protected CommandBase followPathAndPlaceUppercube3(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 2", swerveDrive),
                new UpperScoring()
        );
    }

    protected CommandBase followPathAndPlaceMiddlecube3(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 2", swerveDrive),
                new MiddleScoring()
        );
    }

    protected CommandBase placeUpperCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    protected CommandBase placeMiddleCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3b2 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }
}
