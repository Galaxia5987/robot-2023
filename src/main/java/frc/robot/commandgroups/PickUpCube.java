package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;

public class PickUpCube extends ParallelCommandGroup {

    public PickUpCube() {
        Arm arm = Arm.getInstance();
        Gripper gripper = Gripper.getInstance();
        Intake intake = Intake.getInstance();

        addCommands(
                new GetArmIntoRobot(),
                new WaitUntilCommand(() -> !arm.armIsOutOfFrame()).andThen(new InstantCommand(gripper::open, gripper))
                .andThen(new Feed(false))
        );
    }
}

