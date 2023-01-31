package frc.robot.commadGroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;

import java.lang.management.GarbageCollectorMXBean;

public class PickFromFeeder extends SequentialCommandGroup {
    public PickFromFeeder(Arm arm, Translation2d armStartPosition, Translation2d armEndPosition, Gripper gripper){
        addCommands(
                //TODO: use the drivetrain commands in order to set the robot to the right angle and position
                new SetArmsPosition(armStartPosition).alongWith(new InstantCommand(()-> gripper.open())),
                new InstantCommand(()-> gripper.close()),
                new SetArmsPosition(armEndPosition)
        );
    }
}
