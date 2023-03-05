package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;

public class CheckArmPositions extends SequentialCommandGroup {
    public CheckArmPositions() {
        new GetArmIntoRobot().withTimeout(3);
        new WaitCommand(1);
        new SetArmsPositionAngular(() -> ArmConstants.STARTING_POSITION, 0.02).withTimeout(3);
    }
}
