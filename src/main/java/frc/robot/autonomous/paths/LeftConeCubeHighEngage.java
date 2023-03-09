package frc.robot.autonomous.paths;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;

/**
 * This class contains all parts of the path LeftConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class LeftConeCubeHighEngage extends SequentialCommandGroup {
    public LeftConeCubeHighEngage() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();

        addCommands(
                new LeftConeCubeHigh(),

                FollowPath.loadTrajectory("LeftConeCubeHigh blue charge")
                        .andThen(
                                new DriveTillPitch(10.5, 1),
                                new RunCommand(() -> swerveDrive.drive(
                                        new DriveSignal(
                                                1,
                                                0,
                                                0,
                                                new Translation2d(),
                                                true
                                        )
                                ), swerveDrive).withTimeout(1.35)
                        ).alongWith(
                                new ReturnArm().withTimeout(1).andThen(new GetArmIntoRobot()),
                                new ReturnIntake()),

                new RunCommand(swerveDrive::lock)

        );
    }
}
