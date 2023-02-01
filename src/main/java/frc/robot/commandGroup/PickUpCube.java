package frc.robot.commandGroup;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Command.Feed;
import frc.robot.subsystems.intake.Intake;

public class PickUpCube extends SequentialCommandGroup {
    Gripper gripper = Gripper.getInstance();
    Intake intake = Intake.getInstance();

    public PickUpCube( double intakePower, Translation2d armPosition, BeamBreaker beamBreaker) {
        addCommands(
                // Feeds the cube into the Intake until the BeamBreaker sends a false signal
                new Feed(intakePower, intake, beamBreaker),
                //Sets the arm position into the inside space of the Robot and meanwhile opens the gripper
                new SetArmsPosition(armPosition).alongWith(new InstantCommand(()-> gripper.open())),
                //Closes the Gripper after it is set on the position of the cube.
                new InstantCommand(() -> gripper.close())
        );
    }
}

