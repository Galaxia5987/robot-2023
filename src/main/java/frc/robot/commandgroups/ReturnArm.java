package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;

public class ReturnArm extends SequentialCommandGroup {

    public ReturnArm(boolean feederPosition) {

        addCommands(
                feederPosition ? new SetArmsPositionAngular(() -> ArmConstants.FEEDER_POSITION) :
                new SetArmsPositionAngular(() -> ArmConstants.OUT_ROBOT2)
        );
    }
}
