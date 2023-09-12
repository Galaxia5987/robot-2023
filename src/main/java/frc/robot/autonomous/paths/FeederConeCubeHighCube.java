package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoFunctions;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;

/**
 * This class contains all parts of the path FeederConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class FeederConeCubeHighCube extends AutoFunctions {
    public FeederConeCubeHighCube() {
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("FeederConeCubeHigh 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(autoBegin(trajectory),

                FollowPath.loadTrajectory("FeederConeCubeHigh 1").alongWith(new PickUpCube().withTimeout(3.3)),

                FollowPath.loadTrajectory("FeederConeCubeHigh 2").alongWith(new ReturnIntake().andThen(new InstantCommand(gripper::close, gripper)).andThen(new ReturnArm().withTimeout(0.65))),

                autoUpperScoring(false),

                FollowPath.loadTrajectory("FeederConeCubeHigh 3").alongWith(new PickUpCube().withTimeout(5)));
    }
}
