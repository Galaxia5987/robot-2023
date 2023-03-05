package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.subsystems.leds.YellowLed;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.AllianceFlipUtil;

/**
 * This class contains all the parts for the path MiddleConeHighEngage.
 * <p>
 * In this path the robot places a cone in the middle grid
 * in the part that is closer to the feeder and goes to the charge station.
 */
public class MiddleConeHighCommunityEngage extends SequentialCommandGroup {

    public MiddleConeHighCommunityEngage() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        Intake intake = Intake.getInstance();
        Gripper gripper = Gripper.getInstance();
        Arm arm = Arm.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MiddleConeHighCommunityFeedEngage 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("MiddleConeHighCommunityFeedEngage 2", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                new InstantCommand(() -> swerveDrive.resetOdometry(
                        AllianceFlipUtil.apply(DriverStation.getAlliance(), trajectory.getInitialPose()))),
                new InstantCommand(() -> swerveDrive.resetOdometry(
                        AllianceFlipUtil.apply(DriverStation.getAlliance(), trajectory1.getInitialPose()))),
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),
                new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),

                new AutonUpperScoring(true),

                new InstantCommand(gripper::open).andThen(gripper::close),

                FollowPath.loadTrajectory("MiddleConeHighCommunityFeedEngage 1").alongWith(new PickUpCube()).withTimeout(3),
                FollowPath.loadTrajectory("MiddleConeHighCommunityFeedEngage 2").andThen(new BalanceOnStation()).andThen(()-> swerveDrive.lock())
        );
    }
}
