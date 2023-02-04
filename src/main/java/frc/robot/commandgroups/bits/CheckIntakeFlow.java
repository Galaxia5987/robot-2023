package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Command.Feed;

public class CheckIntakeFlow extends SequentialCommandGroup {
    public CheckIntakeFlow() {
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new Feed(0.5).withTimeout(3),
                new WaitCommand(1),
                new Feed(0.5).withTimeout(3),
                new WaitCommand(1),
                new Feed(0.5).withTimeout(3),
                new WaitCommand(1),
                new Feed(0.5).withTimeout(3),
                new WaitCommand(1),
                new InstantCommand(gripper::open, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::close, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::open, gripper)
        );
    }
}

