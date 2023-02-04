package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Command.Feed;

public class PickUpCube extends SequentialCommandGroup {

    public PickUpCube(double intakePower, Translation2d armPosition) {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new Feed(intakePower),
                new SetArmsPosition(armPosition).alongWith(new InstantCommand(gripper::open, gripper)),
                new InstantCommand(gripper::close, gripper)
        );
    }
}

