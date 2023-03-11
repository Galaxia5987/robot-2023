package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.leds.PurpleLed;
import frc.robot.utils.AllianceFlipUtil;

public class RightConeCubeHighRed extends SequentialCommandGroup {
    public RightConeCubeHighRed() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("RightConeCubeHigh red 1",
                new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO,
                        SwerveConstants.MAX_ACCELERATION_AUTO));


        addCommands(
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
                new InstantCommand(() -> swerveDrive.resetOdometry(
                        AllianceFlipUtil.apply(DriverStation.getAlliance(), trajectory.getInitialPose()))),

                new InstantCommand(gripper::close, gripper).withTimeout(1),

                new AutonUpperScoring(true),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(2),

                new PurpleLed(),

                FollowPath.loadTrajectory("RightConeCubeHigh red 1")
                        .alongWith(
                                new PickUpCube().withTimeout(4.5)),

                FollowPath.loadTrajectory("RightConeCubeHigh red 2")
                        .alongWith(new ReturnIntake()
                                .andThen(new InstantCommand(gripper::close, gripper))
                                .andThen(new ReturnArm().withTimeout(1))),

                new AutonUpperScoring(false),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm()
        );
    }
}
