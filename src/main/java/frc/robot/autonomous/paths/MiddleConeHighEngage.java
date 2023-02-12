package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.vision.Limelight;

/**
 * This class contains all the parts for the path MiddleConeHighEngage.
 * <p>
 * In this path the robot places a cone in the middle grid
 * in the part that is closer to the feeder and goes to the charge station.
 */
public class MiddleConeHighEngage extends SequentialCommandGroup {

    public MiddleConeHighEngage() {
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setTapePipeline, limelight),
                new UpperScoring(),
                FollowPath.loadTrajectory(".pathplanne/MiddleConeHighEngage blue"),
                new BalanceOnStation()
        );
    }
}
