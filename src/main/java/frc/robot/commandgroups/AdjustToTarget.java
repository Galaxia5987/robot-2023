package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.vision.Limelight;

public class AdjustToTarget extends SequentialCommandGroup {

    public AdjustToTarget(boolean rightSide, boolean useHorizontalOffset) {
        var limelight = Limelight.getInstance();

        addCommands(
                new ConditionalCommand(
                        new AdjustToTape(),
                        new AdjustToAprilTag(rightSide, useHorizontalOffset),
                        () -> limelight.getPipeline() == Limelight.Pipeline.REFLECTIVE_TAPE_PIPELINE
                )
        );
    }
}
