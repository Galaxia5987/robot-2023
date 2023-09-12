package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoFunctions;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;

/**
 * This class contains all the parts of the path LeftConeHighRun.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder
 * and goes to park in the opposite alliance's loading zone.
 */
public class FeederConeHighRun extends AutoFunctions {
    public FeederConeHighRun() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("FeederRun 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                autoBegin(trajectory),

                new ReturnArm().withTimeout(3),

                FollowPath.loadTrajectory("FeederRun 1")
        );
    }
}
