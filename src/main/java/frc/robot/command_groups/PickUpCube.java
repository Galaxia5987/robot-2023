package frc.robot.command_groups;

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
                // Feeds the cube into the Intake until the BeamBreaker sends a false signal
                new Feed(intakePower),
                //Sets the arm position into the inside space of the Robot and meanwhile opens the gripper
                new SetArmsPosition(armPosition).alongWith(new InstantCommand(gripper::open, gripper)),
                //Closes the Gripper after it is set on the position of the cube.
                new InstantCommand(gripper::close, gripper)
        );
    }
}

