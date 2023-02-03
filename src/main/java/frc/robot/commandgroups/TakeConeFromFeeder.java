package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandgroups.AdjustToAprilTag;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;

public class TakeConeFromFeeder extends SequentialCommandGroup {

    public TakeConeFromFeeder(boolean rightSide) {
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new AdjustToTarget(rightSide, true)
                        .alongWith(new InstantCommand(gripper::open)),
                new SetArmsPosition(ArmConstants.FEEDER_POSITION),
                new InstantCommand(gripper::close),
                new SetArmsPosition(ArmConstants.RETRACTED_POSITION)
        );
    }
}
