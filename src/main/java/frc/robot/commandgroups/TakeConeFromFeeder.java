package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.arm.commands.SetArmsPositionLinear;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;

public class TakeConeFromFeeder extends SequentialCommandGroup {

    public TakeConeFromFeeder(boolean rightSide) {
        Gripper gripper = Gripper.getInstance();
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setAprilTagsPipeline, limelight),
                new AdjustToAprilTag(rightSide, true)
                        .alongWith(new InstantCommand(gripper::open, gripper)),
                new SetArmsPositionAngular(ArmConstants.FEEDER_POSITION),
                new InstantCommand(gripper::close),
                new SetArmsPositionAngular(ArmConstants.RETRACTED_POSITION)
        );
    }
}
