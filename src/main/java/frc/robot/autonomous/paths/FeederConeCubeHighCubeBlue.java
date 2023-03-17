package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
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

import static frc.robot.subsystems.intake.commands.Retract.Mode.DOWN;

/**
 * This class contains all parts of the path LeftConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class FeederConeCubeHighCubeBlue extends SequentialCommandGroup {
    public FeederConeCubeHighCubeBlue() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new InstantCommand(gripper::close, gripper).withTimeout(1),

                new Retract(DOWN).withTimeout(0.35)
                        .andThen(new AutonUpperScoring(true)),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(1),

                new PurpleLed(),

                FollowPath.loadTrajectory("LeftConeCubeHigh blue 1",
                        FollowPath.resetCommand(swerveDrive, gyroscope)).alongWith(
                        new PickUpCube().withTimeout(3.8)
                ),

                FollowPath.loadTrajectory("LeftConeCubeHigh blue 2")
                        .alongWith(
                                new ReturnIntake()
                                        .andThen(new InstantCommand(gripper::close, gripper))
                                        .andThen(new ReturnArm().withTimeout(1))
                        ),

                new AutonUpperScoring(false),
                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(1.5)

//                FollowPath.loadTrajectory("LeftConeCubeHigh blue 3").alongWith(new PickUpCube().withTimeout(5))
        );
    }
}
