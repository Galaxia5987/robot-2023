package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.PickUpCube;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.commandgroups.Feed;
import frc.robot.subsystems.intake.Intake;

public class CheckIntakeFlow extends SequentialCommandGroup {
    public CheckIntakeFlow() {
        Gripper gripper = Gripper.getInstance();
        Intake intake = Intake.getInstance();

        addCommands(
                new ReturnIntake(),
                new PickUpCube().withTimeout(3)
                        .andThen(new ReturnIntake()),
                new WaitCommand(1),
                new PickUpCube().withTimeout(3)
                        .andThen(new ReturnIntake()),
                new WaitCommand(1),
                new PickUpCube().withTimeout(3)
                        .andThen(new ReturnIntake()),
                intake.run(0.5).withTimeout(3)
                        .finallyDo((b) -> intake.setPower(0)),
                new WaitCommand(1),
                new InstantCommand(gripper::toggle, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::toggle, gripper),
                new WaitCommand(1),
                new InstantCommand(gripper::toggle, gripper)
        );
    }
}

