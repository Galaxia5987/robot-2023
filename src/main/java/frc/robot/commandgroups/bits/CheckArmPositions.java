package frc.robot.commandgroups.bits;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;

public class CheckArmPositions extends SequentialCommandGroup {
    public CheckArmPositions() {
        new SetArmsPositionAngular(new Translation2d(0, 0), 0.02);
        new WaitCommand(1);
        new SetArmsPositionAngular(new Translation2d(0, 0), 0.02);
        new WaitCommand(1);
        new SetArmsPositionAngular(new Translation2d(0, 0), 0.02);
        new WaitCommand(1);
        new SetArmsPositionAngular(ArmConstants.ABOVE_GAME_PIECE, 0.02);
    }
}
