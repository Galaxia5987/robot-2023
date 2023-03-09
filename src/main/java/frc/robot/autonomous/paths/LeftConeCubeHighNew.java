package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.leds.PurpleLed;
import frc.robot.utils.AllianceFlipUtil;

/**
 * This class contains all parts of the path LeftConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class LeftConeCubeHighNew extends SequentialCommandGroup {
    public LeftConeCubeHighNew() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("LeftConeCubeHigh blue 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                new InstantCommand(() -> swerveDrive.resetOdometry(
                        AllianceFlipUtil.apply(DriverStation.getAlliance(), trajectory.getInitialPose()))),
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
                new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),

                //new InstantCommand(gripper::close, gripper).withTimeout(1),

                //new AutonUpperScoring(true),

                //new InstantCommand(gripper::open, gripper),

                new PurpleLed(),

                FollowPath.loadTrajectory("LeftConeCubeHigh blue 1").alongWith(
                        new PickUpCube().withTimeout(4)),

                FollowPath.loadTrajectory("LeftConeCubeHigh blue 2")
                        .alongWith(new ReturnIntake()),
                //.andThen(new InstantCommand(gripper::close, gripper))
                //.andThen(new ReturnArm())),

                // new AutonUpperScoring(false),

                //new InstantCommand(gripper::open, gripper),

                //new ReturnArm(),

                FollowPath.loadTrajectory("LeftConeCubeHigh blue 3").andThen(new PickUpCube()).withTimeout(1.5)
        );
    }
}
