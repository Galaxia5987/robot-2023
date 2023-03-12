package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.gripper.Gripper;

public class ArmAxisControlWithClosing extends SequentialCommandGroup {

    public ArmAxisControlWithClosing() {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new ArmAxisControl(1, 0.02, 0),
                new InstantCommand(gripper::close)
        );
    }
}
