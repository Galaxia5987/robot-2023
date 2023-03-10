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
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.Retract;

public class RightConeCubeHighCube extends SequentialCommandGroup {
    public RightConeCubeHighCube(){
        Gripper gripper = Gripper.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("RightConeCubeHighCube blue 1",
                new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                new InstantCommand(()-> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
                new InstantCommand(()-> swerveDrive.resetOdometry(trajectory.getInitialHolonomicPose())),
                new AutonUpperScoring(true),
                new InstantCommand(gripper::open),
                new ReturnArm().withTimeout(1),
                FollowPath.loadTrajectory("RightConeCubeHighCube blue 1").alongWith(new PickUpCube()),
                new InstantCommand(gripper::close),
                FollowPath.loadTrajectory("RightConeCubeHighCube blue 2"),
                new AutonUpperScoring(false),
                new InstantCommand(gripper::open),
                FollowPath.loadTrajectory("RightConeCubeHighCube blue 3").alongWith(new PickUpCube())
        );
    }
}
