package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

public class ReturnArm extends SequentialCommandGroup {

    public ReturnArm() {
        addCommands(
                new SetArmsPositionAngular(() -> ArmConstants.OUT_ROBOT2, 0.1)
                        .raceWith(new RunCommand(() -> Intake.getInstance().setAnglePower(0.05))
                                .finallyDo((b) -> Intake.getInstance().setAnglePower(0)))
        );
    }
}
