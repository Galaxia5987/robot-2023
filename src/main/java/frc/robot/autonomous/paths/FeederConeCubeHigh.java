package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.autonomous.ResetAuto;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.subsystems.leds.PurpleLed;

import static frc.robot.subsystems.intake.commands.Retract.Mode.DOWN;

public class FeederConeCubeHigh extends SequentialCommandGroup {
    public FeederConeCubeHigh() {
        Gyroscope gyroscope = Gyroscope.getInstance();
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new ResetAuto(),

                new Retract(DOWN).withTimeout(0.35)
                        .andThen(new AutonUpperScoring(true)),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(0.75),

                new PurpleLed(),

                FollowPath.loadTrajectory("FeederConeCubeHigh 1",
                        FollowPath.resetCommand(swerveDrive, gyroscope)).alongWith(
                        new PickUpCube().withTimeout(3.8)
                ),

                FollowPath.loadTrajectory("FeederConeCubeHigh 2")
                        .alongWith(
                                new ReturnIntake()
                                        .andThen(new InstantCommand(gripper::close, gripper))
                                        .andThen(new ReturnArm().withTimeout(0.65))
                        ),

                new AutonUpperScoring(false),
                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(0.65)
        );
    }
}