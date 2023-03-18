package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;

/**
 * This class contains all parts of the path FeederConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class FeederConeCubeHighCube extends SequentialCommandGroup {
    public FeederConeCubeHighCube() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new FeederConeCubeHigh(),

                FollowPath.loadTrajectory("FeederConeCubeHigh 3").alongWith(new PickUpCube().withTimeout(5))
        );
    }
}
