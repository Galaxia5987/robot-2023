package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class MiddleConeHighCubeEngage extends SequentialCommandGroup {
    public MiddleConeHighCubeEngage() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MiddleConeHighCubeEngage blue 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));


        addCommands(
                new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
                new InstantCommand(gripper::open),
                new AutonUpperScoring(true).andThen(new InstantCommand(gripper::close)),
                

                );
    }
}
