package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.commandgroups.Feed;
import frc.robot.subsystems.intake.Intake;

public class CheckIntakeFlow extends SequentialCommandGroup {
    public CheckIntakeFlow() {
        Gripper gripper = Gripper.getInstance();
        Intake intake = Intake.getInstance();

        addCommands(
                intake.run(0.5).withTimeout(3),
                new WaitCommand(1),
                intake.run(0.5).withTimeout(3),
                new WaitCommand(1),
                intake.run(0.5).withTimeout(3),
                new WaitCommand(1),
                intake.run(0.5).withTimeout(3),
                new WaitCommand(1),
                new InstantCommand(gripper::open, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::close, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::open, gripper)
        );
    }
}

