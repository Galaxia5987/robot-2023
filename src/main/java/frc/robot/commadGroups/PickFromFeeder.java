package frc.robot.commadGroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;


public class PickFromFeeder extends SequentialCommandGroup {
    private Gripper gripper = Gripper.getInstance();
    public PickFromFeeder(Translation2d armStartPosition, Translation2d armEndPosition){
        addCommands(
                new SetArmsPosition(armStartPosition).alongWith(new InstantCommand(()-> gripper.open())),
                new InstantCommand(()-> gripper.close()),
                new SetArmsPosition(armEndPosition)
        );
    }
}
