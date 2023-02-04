package frc.robot.command_groups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Command.Feed;
import frc.robot.subsystems.intake.ConstantsIntake;

public class PickUpCube extends SequentialCommandGroup {

    public PickUpCube() {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                // Feeds the cube into the Intake until the BeamBreaker sends a false signal
                new Feed(ConstantsIntake.INTAKE_POWER),
                //Sets the arm position into the inside space of the Robot and meanwhile opens the gripper
                new SetArmsPosition(ArmConstants.ABOVE_GAME_PIECE).alongWith(new InstantCommand(gripper::open, gripper)),
                //Closes the Gripper after it is set on the position of the cube.
                new InstantCommand(gripper::close, gripper)
        );
    }
}

