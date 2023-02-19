package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.arm.commands.SetElbowAngle;
import frc.robot.subsystems.arm.commands.SetShoulderAngle;

public class GetArmIntoRobot extends SequentialCommandGroup {

    public GetArmIntoRobot() {
        Arm arm = Arm.getInstance();

        addCommands(
                new SetArmsPositionAngular(() -> ArmConstants.OUT_ROBOT1, 0.05),
                new SetShoulderAngle(129).alongWith(new InstantCommand(arm::stop)),
                new SetElbowAngle(329),
                arm.holdCommand()
        );
    }
}
