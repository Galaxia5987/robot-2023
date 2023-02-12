package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;

public class MidScoring extends SequentialCommandGroup {

    public MidScoring() {
        Limelight limelight = Limelight.getInstance();
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new AdjustToTarget(true, false),
                new SetArmsPosition(() -> limelight.getPipeline() == Limelight.Pipeline.REFLECTIVE_TAPE_PIPELINE ?
                        ArmConstants.MIDDLE_CONE_SCORING :
                        ArmConstants.MIDDLE_CUBE_SCORING),
                new InstantCommand(gripper::open, gripper),
                new WaitCommand(ArmConstants.WAIT_TIME),
                new SetArmsPosition(ArmConstants.RETRACTED_POSITION)
        );
    }
}
