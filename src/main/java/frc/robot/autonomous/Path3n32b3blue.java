package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.MiddleScoring;
import frc.robot.commandGroups.PickUpCube;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.intake.ConstantsIntake;

public class Path3n32b3blue extends AutonomousPaths{
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public CommandBase placeUpperCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    public CommandBase placeMiddleCone32FollowPathAndPickUpCube() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 1", swerveDrive),
                new PickUpCube(ConstantsIntake.INTAKE_POWER, ArmConstants.ABOVE_GAME_PIECE)
        );
    }

    public CommandBase followPathAndPlaceUppercube3(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 2", swerveDrive),
                new UpperScoring()
        );
    }

    public CommandBase followPathAndPlaceMiddlecube3(){
        return new SequentialCommandGroup(
                FollowPath.loadTrajectory(".pathplanner/3n32b3 blue 2", swerveDrive),
                new MiddleScoring()
        );
    }
}
