package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.leds.YellowLed;
import frc.robot.subsystems.vision.Limelight;

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
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MiddleConeHighEngage blue", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
                new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),

                new YellowLed(),

                new UpperScoring().withTimeout(5),

                new InstantCommand(gripper::open),

                new DriveTillPitch(10),

                new RunCommand(() -> swerveDrive.drive(
                        new DriveSignal(
                                2,
                                0,
                                SwerveConstants.AUTO_ROTATION_Kf,
                                new Translation2d(),
                                true
                        )
                ), swerveDrive).withTimeout(1.5)
                        .alongWith(new ReturnArm()),

                new RunCommand(swerveDrive::lock)
        );
    }
}
