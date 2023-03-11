package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.subsystems.leds.PurpleLed;

public class LeftConeCubeHighCubeRed extends SequentialCommandGroup {
    public LeftConeCubeHighCubeRed() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("LeftConeCubeHigh red 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),

                new InstantCommand(gripper::close, gripper).withTimeout(1),

                new Retract(false).withTimeout(0.35)
                        .andThen(new AutonUpperScoring(true)),

                new InstantCommand(gripper::open, gripper),

                new PurpleLed(),

                FollowPath.loadTrajectory("LeftConeCubeHigh red 1").alongWith(
                        new PickUpCube().withTimeout(3.7)),

                FollowPath.loadTrajectory("LeftConeCubeHigh red 2")
                        .alongWith(new ReturnIntake()
                                .andThen(new InstantCommand(gripper::close, gripper))
                                .andThen(new ReturnArm()
                                        .withTimeout(1))),

                new AutonUpperScoring(false),
                new InstantCommand(gripper::open, gripper),

                FollowPath.loadTrajectory("LeftConeCubeHigh red 3").alongWith(new PickUpCube().withTimeout(5)));
    }
}
