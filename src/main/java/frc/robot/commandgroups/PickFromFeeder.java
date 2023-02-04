package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;


public class PickFromFeeder extends SequentialCommandGroup {
    public PickFromFeeder(Translation2d armStartPosition, Translation2d armEndPosition){
        Gripper gripper = Gripper.getInstance();
        Limelight limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(limelight::setAprilTagsPipeline, limelight),
                new SetArmsPosition(armStartPosition).alongWith(new InstantCommand(gripper::open, gripper)),
                new InstantCommand(gripper::close, gripper),
                new SetArmsPosition(armEndPosition)
        );
    }
}
