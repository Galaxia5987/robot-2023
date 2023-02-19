package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.gripper.Gripper;

public class ReturnArmCube extends SequentialCommandGroup {

    public ReturnArmCube(boolean openGripperAtEnd) {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new InstantCommand(openGripperAtEnd ? gripper::open : gripper::close, gripper),
                new SetArmsPositionAngular(ArmConstants.OUT_ROBOT2)
        );
    }
}
