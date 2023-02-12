package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.commands.Feed;

public class PickUpCube extends SequentialCommandGroup {

    public PickUpCube() {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new InstantCommand(gripper::open, gripper),
                new Feed(IntakeConstants.INTAKE_POWER),
                new SetArmsPosition(ArmConstants.ABOVE_GAME_PIECE),
                new InstantCommand(gripper::close, gripper)
        );
    }
}

