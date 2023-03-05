package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.BalanceOnStation;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.Intake;


public class MiddleConeHighCommunityFeedEngage extends SequentialCommandGroup {

    public MiddleConeHighCommunityFeedEngage() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        Intake intake = Intake.getInstance();
        Gripper gripper = Gripper.getInstance();
        Arm arm = Arm.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("RightConeCubeHigh blue 1",
                new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO,
                        SwerveConstants.MAX_ACCELERATION_AUTO));
        addCommands(

                new AutonUpperScoring(true),

//                new InstantCommand(gripper::open).withTimeout(3).andThen(gripper::close),
//                new GetArmIntoRobot(),
//                new DriveTillPitch(-10.5, 1).andThen(new DriveTillPitch(0, 1))
//                        .andThen(new DriveTillPitch(10.5, 1)).andThen(new DriveTillPitch(0, 1)),
//                new InstantCommand(()-> swerveDrive.drive(new DriveSignal(1, 0, 0, new Translation2d(0,0),false))).alongWith(new PickUpCube()).withTimeout(3),
//                new DriveTillPitch(10.5, -1).andThen(new DriveTillPitch(0, -1)).andThen(new DriveTillPitch(-10.5, -1)).andThen(new DriveTillPitch(0, -1)),
//                new InstantCommand(()-> swerveDrive.drive(new DriveSignal(1, 0, 0, new Translation2d(0,0),false))).alongWith(new PickUpCube()).withTimeout(3),

                );
    }
}
