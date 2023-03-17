package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.autonomous.ResetAuto;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.leds.PurpleLed;

/**
 * This class contains all the parts to the path RightConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is the furthest away from the feeder,
 * goes to pick up a cube (the one furthest from the feeder)
 * and returns to place it in the same grid.
 */
public class BumperConeCubeHigh extends SequentialCommandGroup {

    public BumperConeCubeHigh() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new ResetAuto(),

                new AutonUpperScoring(true),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(0.75),

                new PurpleLed(),

                FollowPath.loadTrajectory("BumperConeCubeHigh 1", FollowPath.resetCommand(swerveDrive, gyroscope))
                        .alongWith(
                                new PickUpCube().withTimeout(3.5)),

                FollowPath.loadTrajectory("BumperConeCubeHigh 2")
                        .alongWith(new ReturnIntake()
                                .andThen(new InstantCommand(gripper::close, gripper))
                                .andThen(new ReturnArm().withTimeout(0.75))),

                new AutonUpperScoring(false),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm()
        );
    }
}
