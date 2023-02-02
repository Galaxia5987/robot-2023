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
    private final Arm arm;
    private final Gripper gripper;
    private final SwerveDrive swerveDrive;
    private final Intake intake;
    private final FollowPath followPath;

    public AutonomousPaths(Arm arm, Gripper gripper, SwerveDrive swerveDrive, Intake intake, FollowPath followPath) {
        this.arm = arm;
        this.gripper = gripper;
        this.swerveDrive = swerveDrive;
        this.intake = intake;
        this.followPath = followPath;
    }

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

    protected CommandBase placeUpperCone11AndRun() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11r blue", swerveDrive)
        );
    }

    protected CommandBase placeMiddleCone21AndRun() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanner/1n11r blue", swerveDrive)
        );
    }

    protected CommandBase placeUpperCone21AndCharge() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/2n21g", swerveDrive),
                new BalanceOnStation()
        );
    }

    protected CommandBase placeMiddleCone21AndCharge() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanne/2n21g", swerveDrive),
                new BalanceOnStation()
        );
    }

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
}
