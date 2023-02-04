package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;

public class TakeConeFromFeeder extends SequentialCommandGroup {

    public TakeConeFromFeeder(boolean rightSide) {
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new AdjustToAprilTag(rightSide, true)
                        .alongWith(new InstantCommand(gripper::open)),
                new SetArmsPosition(ArmConstants.FEEDER_POSITION),
                new InstantCommand(gripper::close),
                new SetArmsPosition(ArmConstants.RETRACTED_POSITION)
        );
    }
}
