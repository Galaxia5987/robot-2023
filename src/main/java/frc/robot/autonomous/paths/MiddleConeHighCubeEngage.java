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
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.utils.AllianceFlipUtil;

import java.sql.Struct;

public class MiddleConeHighCubeEngage extends SequentialCommandGroup {
    public MiddleConeHighCubeEngage() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MiddleConeHighCubeEngage blue 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));


        addCommands(
                new InstantCommand(() -> swerveDrive.resetOdometry(
                        AllianceFlipUtil.apply(DriverStation.getAlliance(), trajectory.getInitialPose()))),
                new InstantCommand(() -> gyroscope.resetYaw(trajectory.getInitialHolonomicPose().getRotation())),

                new Retract(false).withTimeout(0.35)
                        .andThen(new AutonUpperScoring(true)),

                new InstantCommand(gripper::open),

                new DriveTillPitch(10.5, 1)
                        .alongWith(new ReturnArm()),

                new RunCommand(()-> swerveDrive.drive(new DriveSignal(
                        1.2,0,0, new Translation2d(), true)), swerveDrive)
                        .alongWith(new PickUpCube())
                        .withTimeout(2.2),

                        new RunCommand(()-> swerveDrive.drive(new DriveSignal(
                        -1.3,0,0, new Translation2d(), true)), swerveDrive).withTimeout(3));
    }
}
