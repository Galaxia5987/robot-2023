package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;

public class BumperConeCubeHighEngage extends SequentialCommandGroup {
    public BumperConeCubeHighEngage() {
        addCommands(
                new BumperConeCubeHigh(),
                FollowPath.loadTrajectory("BumperConeCubeHighEngage"),
                new Engage(true, false)
        );
    }
}
