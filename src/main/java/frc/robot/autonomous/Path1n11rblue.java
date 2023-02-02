package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.MiddleScoring;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Path1n11rblue extends AutonomousPaths{
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

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
}
