package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.commands.Retract;

public class ResetAuto extends SequentialCommandGroup {

    public ResetAuto() {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new Retract(Retract.Mode.UP),
                new InstantCommand(gripper::close, gripper),
                new WaitCommand(0.02)
        );
    }
}
