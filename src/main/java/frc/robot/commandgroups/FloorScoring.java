package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.gripper.Gripper;

public class FloorScoring extends SequentialCommandGroup {

    public FloorScoring() {
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new AdjustToTarget(true, false),
                new SetArmsPositionAngular(() -> ArmConstants.FLOOR_SCORING, 0.02),
                new InstantCommand(gripper::open, gripper),
                new WaitCommand(ArmConstants.WAIT_TIME),
                new SetArmsPositionAngular(() -> ArmConstants.RETRACTED_POSITION, 0.02)
        );
    }
}
