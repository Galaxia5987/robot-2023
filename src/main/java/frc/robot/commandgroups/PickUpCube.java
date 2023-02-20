package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class PickUpCube extends SequentialCommandGroup {

    public PickUpCube() {
        Gripper gripper = Gripper.getInstance();
        Intake intake = Intake.getInstance();

        addCommands(
                new InstantCommand(gripper::open, gripper),
                intake.run(IntakeConstants.INTAKE_POWER),
                new SetArmsPositionAngular(() -> ArmConstants.ABOVE_GAME_PIECE),
                new InstantCommand(gripper::close, gripper)
        );
    }
}

