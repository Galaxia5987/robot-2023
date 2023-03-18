package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;

public class FeederConeCubeHighEngage extends SequentialCommandGroup {

    public FeederConeCubeHighEngage() {

        addCommands(
                new FeederConeCubeHigh(),

                FollowPath.loadTrajectory("FeederConeCubeHigh engage").withTimeout(1.25),

                new Engage(true, true)
        );
    }
}
