package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;


public class PickFromFeeder extends SequentialCommandGroup {
    public PickFromFeeder(Translation2d armStartPosition, Translation2d armEndPosition){
        Gripper gripper = Gripper.getInstance();
        addCommands(
                new SetArmsPosition(armStartPosition).alongWith(new InstantCommand(gripper::open, gripper)),
                new InstantCommand(gripper::close, gripper),
                new SetArmsPosition(armEndPosition)
        );
    }
}
