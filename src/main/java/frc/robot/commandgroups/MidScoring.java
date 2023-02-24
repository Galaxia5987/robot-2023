package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MidScoring extends SequentialCommandGroup {

    public MidScoring(DoubleSupplier xSupplier, BooleanSupplier cone) {
        Limelight limelight = Limelight.getInstance();
        Arm arm = Arm.getInstance();

        addCommands(
                new ConditionalCommand(
                        new InstantCommand(limelight::setTapeMiddlePipeline),
                        new InstantCommand(limelight::setAprilTagsPipeline),
                        cone
                )


//                new ConditionalCommand(
//                        new SetArmsPositionAngular(() -> ArmConstants.MIDDLE_CONE_SCORING1, 0.05),
//                        new InstantCommand(),
//                        () -> arm.getElbowJointAngle().getDegrees() > 180)
//                        .andThen(new SetArmsPositionAngular(() -> ArmConstants.MIDDLE_CONE_SCORING2))
//                        .alongWith(new ConditionalCommand(
//                                new AdjustToTarget(Math.toRadians(1.44), Math.toRadians(16.62 + 1.44)),
//                                new AdjustToAprilTag(xSupplier),
//                                cone
//                        ))
        );
    }
}
