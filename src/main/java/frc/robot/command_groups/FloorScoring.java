package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionConstants;

public class FloorScoring extends SequentialCommandGroup {

    public FloorScoring(){
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new AdjustToTarget(true, false),
                new SetArmsPosition(ArmConstants.FLOOR_SCORING),
                new InstantCommand(gripper::open, gripper),
                new WaitCommand(ArmConstants.WAIT_TIME),
                new SetArmsPosition(ArmConstants.RETRACTED_POSITION)
        );
    }
}
