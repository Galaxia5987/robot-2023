package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;

public class UpperScoring extends SequentialCommandGroup {

    public UpperScoring() {
        Limelight limelight = Limelight.getInstance();
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new AdjustToTarget(true, false),
                new SetArmsPosition(() -> limelight.getPipeline() == Limelight.Pipeline.REFLECTIVE_TAPE_PIPELINE ?
                        ArmConstants.UPPER_CONE_SCORING :
                        ArmConstants.UPPER_CUBE_SCORING),
                new InstantCommand(gripper::open),
                new SetArmsPosition(ArmConstants.RETRACTED_POSITION)
        );
    }
}
