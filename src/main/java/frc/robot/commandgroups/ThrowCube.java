package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.gripper.Gripper;

public class ThrowCube extends SequentialCommandGroup {
    Gripper gripper = Gripper.getInstance();
    public ThrowCube(){
        addCommands(
                new ReturnArm().withTimeout(0.65),
                new UpperScoring().withTimeout(0.565),
                new InstantCommand(gripper::open, gripper)

        );
    }
}
