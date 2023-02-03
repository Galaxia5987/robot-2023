package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Command.Feed;
import frc.robot.subsystems.intake.ConstantsIntake;
import frc.robot.subsystems.leds.Leds;

public class PickUpCube extends SequentialCommandGroup {

    public PickUpCube(boolean cone, boolean rightSide) {
        Gripper gripper = Gripper.getInstance();
        Leds leds = Leds.getInstance();

        addCommands(
                new InstantCommand(cone ? leds::setYellow : leds::setPurple, leds),
                new Feed(ConstantsIntake.INTAKE_POWER),
                new InstantCommand(gripper::open, gripper),
                new SetArmsPosition(ArmConstants.ABOVE_GAME_PIECE),
                new InstantCommand(gripper::close, gripper)
        );
    }
}

