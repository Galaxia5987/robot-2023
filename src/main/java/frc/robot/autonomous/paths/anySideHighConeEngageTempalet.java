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
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.Intake;

import java.awt.*;

public class anySideHighConeEngageTempalet extends SequentialCommandGroup {
    public anySideHighConeEngageTempalet() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        Intake intake = Intake.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));
        addCommands(
                new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialPose().getRotation())),
                new InstantCommand(gripper::close)
                        .andThen(new AutonUpperScoring(true)),
                FollowPath.loadTrajectory("add path")
                //new DriveTillPitch(10.5, 1).andThen(new DriveTillPitch(0, 0.5))
        );
    }


}
