package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionConstants;

public class FloorScoring extends SequentialCommandGroup {

    public FloorScoring() {
        Limelight limelight = Limelight.getInstance();
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new ConditionalCommand(
                        new TapeCommandGroup(),
                        new AprilTagCommandGroup(),
                        () -> limelight.getPipeline() == VisionConstants.REFLECTIVE_TAPE_PIPELINE
                ),
                new SetArmsPosition(ArmConstants.FLOOR_SCORING),
                new InstantCommand(gripper::open, gripper),
                new SetArmsPosition(ArmConstants.RETRACTED_POSITION)
        );
    }
}
