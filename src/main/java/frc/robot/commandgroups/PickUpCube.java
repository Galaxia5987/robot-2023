package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPosition;
import frc.robot.subsystems.gripper.Gripper;
<<<<<<< HEAD:src/main/java/frc/robot/commandgroups/PickUpCube.java
import frc.robot.subsystems.intake.Command.Feed;
import frc.robot.subsystems.intake.ConstantsIntake;
import frc.robot.subsystems.leds.Leds;
=======
import frc.robot.subsystems.intake.commands.Feed;
>>>>>>> barel-errors:src/main/java/frc/robot/command_groups/PickUpCube.java

public class PickUpCube extends SequentialCommandGroup {

    public PickUpCube() {
        Gripper gripper = Gripper.getInstance();

        addCommands(
                new InstantCommand(gripper::open, gripper),
                new Feed(ConstantsIntake.INTAKE_POWER),
                new SetArmsPosition(ArmConstants.ABOVE_GAME_PIECE),
                new InstantCommand(gripper::close, gripper)
        );
    }
}

