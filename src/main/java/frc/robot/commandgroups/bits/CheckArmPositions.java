package frc.robot.commandgroups.bits;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionLinear;

public class CheckArmPositions extends SequentialCommandGroup {
    public CheckArmPositions() {
        new SetArmsPositionLinear(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPositionLinear(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPositionLinear(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPositionLinear(ArmConstants.ABOVE_GAME_PIECE);
    }
}
