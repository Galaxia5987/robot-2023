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
import frc.robot.autonomous.ResetAuto;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.subsystems.leds.PurpleLed;
import frc.robot.utils.controllers.DieterController;

import static frc.robot.subsystems.intake.commands.Retract.Mode.DOWN;

/**
 * This class contains all the parts for the path MiddleConeHighEngage.
 * <p>
 * In this path the robot places a cone in the middle grid
 * in the part that is closer to the feeder and goes to the charge station.
 */
public class MiddleConeHighCommunityEngage extends SequentialCommandGroup {
    private final DieterController yawController = new DieterController(3, 0, 0, 0);

    public MiddleConeHighCommunityEngage() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gyroscope gyroscope = Gyroscope.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MiddleConeHighEngage", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                new ResetAuto(),

                FollowPath.resetCommand(swerveDrive, gyroscope).apply(trajectory),

                new InstantCommand(gripper::close),

                new Retract(DOWN).withTimeout(0.35)
                        .andThen(new AutonUpperScoring(true)),

                new InstantCommand(gripper::open),

                new DriveTillPitch(10.5, 1.5)
                        .alongWith(new ReturnArm().withTimeout(1)),

                new PurpleLed(),

                new RunCommand(() -> swerveDrive.drive(
                        new DriveSignal(
                                1.5,
                                0,
                                yawController.calculate(gyroscope.getYaw().getRadians(), 0),
                                new Translation2d(),
                                true
                        )
                ), swerveDrive)
                        .alongWith(new ReturnArm().withTimeout(1))
                        .withTimeout(1.5),

                new Engage(false, true)
        );
    }
}
