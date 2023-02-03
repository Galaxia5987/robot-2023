package frc.robot.command_groups.bits;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;

public class CheckArmPositions extends SequentialCommandGroup {
    public CheckArmPositions() {
        new SetArmsPosition(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPosition(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPosition(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPosition(ArmConstants.ARM_DEFAULT_POSITION);
    }
}
