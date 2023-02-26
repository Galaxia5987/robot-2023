package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.vision.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MidScoring extends SequentialCommandGroup {

    public MidScoring() {
        Arm arm = Arm.getInstance();

        addCommands(
                new ConditionalCommand(
                        new SetArmsPositionAngular(() -> ArmConstants.MIDDLE_CONE_SCORING1, 0.05),
                        new InstantCommand(),
                        () -> arm.getElbowJointAngle().getDegrees() > 180)
                        .andThen(
                                new SetArmsPositionAngular(() -> ArmConstants.MIDDLE_CONE_SCORING2))
        );
    }
}
