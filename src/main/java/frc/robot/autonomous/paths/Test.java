package frc.robot.autonomous.paths;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;

public class Test extends SequentialCommandGroup {
    public Test(){
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("y axis 2", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
        new InstantCommand(()->gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
        new InstantCommand(()-> swerveDrive.resetOdometry(trajectory.getInitialPose())),
        FollowPath.loadTrajectory("y axis 2"));
    }
}
