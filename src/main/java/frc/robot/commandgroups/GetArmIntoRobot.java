package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.HoldArmPosition;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.arm.commands.SetElbowAngle;
import frc.robot.subsystems.arm.commands.SetShoulderAngle;

public class GetArmIntoRobot extends SequentialCommandGroup {

    public GetArmIntoRobot() {
        Arm arm = Arm.getInstance();

        addCommands(
                new ConditionalCommand(
                        new InstantCommand(),
                        new SetArmsPositionAngular(() -> ArmConstants.IN_ROBOT, 0.05),
                        arm::armIsInRobot
                ),
                new SetShoulderAngle(129).alongWith(new InstantCommand(arm::stop)),
                new SetElbowAngle(329),
                new HoldArmPosition()
        );
    }
}
