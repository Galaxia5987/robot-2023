package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandGroups.MiddleScoring;
import frc.robot.commandGroups.UpperScoring;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;

public class Path2b2gBlue {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public CommandBase placeUpperCube2AndCharge() {
        return new SequentialCommandGroup(
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/2nb2g blue", swerveDrive),
                new BalanceOnStation()
        );
    }

    public CommandBase placeMiddleCube2AndCharge() {
        return new SequentialCommandGroup(
                new MiddleScoring(),
                FollowPath.loadTrajectory(".pathplanne/2nb2g blue", swerveDrive),
                new BalanceOnStation()
        );
    }
}
