package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonUpperScoring;
import frc.robot.autonomous.FollowPath;
import frc.robot.autonomous.ResetAuto;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.intake.commands.Retract;
import frc.robot.subsystems.leds.PurpleLed;

import static frc.robot.subsystems.intake.commands.Retract.Mode.DOWN;

public class FeederConeHighTakeCubeEngage extends SequentialCommandGroup {
    Gripper gripper = Gripper.getInstance();
    SwerveDrive swerveDrive = SwerveDrive.getInstance();
    Gyroscope gyroscope = Gyroscope.getInstance();
    public FeederConeHighTakeCubeEngage(){

        addCommands(
                new ResetAuto(),

                new Retract(DOWN).withTimeout(0.35)
                        .andThen(new AutonUpperScoring(true)),

                new InstantCommand(gripper::open, gripper),

                new ReturnArm().withTimeout(0.65),

                new PurpleLed(),

                FollowPath.loadTrajectory("FeederConeCubeHigh 1",
                        FollowPath.resetCommand(swerveDrive, gyroscope)).alongWith(
                        new PickUpCube().withTimeout(3.3)
                ),
                FollowPath.loadTrajectory("FeederConeHighTakeCube engage"),
                new Engage(false, false)
        );
    }
}
