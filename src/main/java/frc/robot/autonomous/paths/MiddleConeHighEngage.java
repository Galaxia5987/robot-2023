package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;

/**
 * This class contains all the parts for the path MiddleConeHighEngage.
 * <p>
 * In this path the robot places a cone in the middle grid
 * in the part that is closer to the feeder and goes to the charge station.
 */
public class MiddleConeHighEngage extends SequentialCommandGroup {

    public MiddleConeHighEngage() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MiddleConeHighEngage", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                FollowPath.resetCommand(swerveDrive, gyroscope).apply(trajectory),

                new Retract(Retract.Mode.DOWN).withTimeout(0.35).andThen(
                        new AutonUpperScoring(true)),

                new InstantCommand(gripper::open),

                new DriveTillPitch(-10.5, 1.5)
                        .alongWith(new ReturnArm().withTimeout(1)),

                new RunCommand(() -> swerveDrive.drive(
                        new DriveSignal(
                                1.5,
                                0,
                                0,
                                new Translation2d(),
                                true
                        )
                ), swerveDrive).alongWith(new GetArmIntoRobot()).withTimeout(SwerveConstants.FORWARD_BALANCE_TIME),

                new RunCommand(swerveDrive::lock)
        );
    }
}
