package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;

public class GetArmOutOfRobot extends SequentialCommandGroup {

    public GetArmOutOfRobot() {
        addCommands(
                new SetArmsPositionAngular(ArmConstants.OUT_ROBOT1, 0.05),
                new SetArmsPositionAngular(ArmConstants.OUT_ROBOT2, 0.05)
        );
    }
}
