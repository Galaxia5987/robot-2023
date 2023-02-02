package frc.robot.commandgroups.bits;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;

public class CheckArmPositions extends SequentialCommandGroup {
    public CheckArmPositions(Arm arm) {
        new SetArmsPosition(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPosition(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPosition(new Translation2d(0, 0));
        new WaitCommand(1);
        new SetArmsPosition(ArmConstants.ARM_DEFAULT_POSITION);
    }
}
