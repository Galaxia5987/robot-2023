package frc.robot.subsystems.commandGroup;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Command.Feed;

public class PickUpCube extends SequentialCommandGroup {
    public PickUpCube(Gripper gripper, double intakePower, Translation2d armPosition) {
        addCommands(
                // Feeds the cube into the Intake until the BeamBreaker sends a false signal
                new Feed(intakePower, 0),
                //Sets the arm position into the inside space of the Robot and meanwhile opens the gripper
                new SetArmsPosition(armPosition).andThen(() -> gripper.open()),
                //Closes the Gripper after it is set on the position of the cube.
                new InstantCommand(() -> gripper.close())
        );
    }
}

