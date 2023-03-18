package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;

public class BumperConeCubeHighCube extends SequentialCommandGroup {

    public BumperConeCubeHighCube() {

        addCommands(
                new BumperConeCubeHigh(),

                FollowPath.loadTrajectory("BumperConeCubeHigh 3")
                        .alongWith(new PickUpCube().withTimeout(5))
        );
    }
}
