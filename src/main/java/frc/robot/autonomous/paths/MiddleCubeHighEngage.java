package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.vision.Limelight;

/**
 * This class contains al the parts for the path MiddleCubeHighEngage.
 * <p>
 * In this path the robot places a cube in the middle grid
 * in the upper part and goes to the charge station.
 */
public class MiddleCubeHighEngage extends SequentialCommandGroup {

    public MiddleCubeHighEngage() {
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setAprilTagsPipeline, limelight),
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/MiddleCubeHighEngage blue"),
                new BalanceOnStation()
        );
    }
}
