package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;

public class GetArmIntoRobot extends SequentialCommandGroup {

    public GetArmIntoRobot() {
        Arm arm = Arm.getInstance();

        addCommands(
                new SetArmsPositionAngular(() -> ArmConstants.IN_ROBOT1, 0.15).unless(arm::armIsInRobot),
                new SetArmsPositionAngular(() -> ArmConstants.IN_ROBOT2, 0.1)
//                new ArmAxisControl(1, 0, -0.02,
//                        0, 0).until(() -> arm.getEndPosition().getY() <= -0.13)
        );
    }
}
