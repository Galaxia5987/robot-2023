package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.gripper.Gripper;

public class ResetAuto extends SequentialCommandGroup {

    public ResetAuto() {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new InstantCommand(gripper::close, gripper)
        );
    }
}
