package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.leds.YellowLed;

public class LeftConeHighRunRed extends SequentialCommandGroup {
    public LeftConeHighRunRed() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("LeftRun red1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
                new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),

                new InstantCommand(gripper::close, gripper).withTimeout(1),
                new YellowLed(),
                new UpperScoring().withTimeout(4),
                new InstantCommand(gripper::open, gripper).withTimeout(1),
                new ReturnArm().withTimeout(3),
                FollowPath.loadTrajectory("LeftRun red1")
        );
    }
}