package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class UpperScoring extends SequentialCommandGroup {

    public UpperScoring(DoubleSupplier xSupplier, BooleanSupplier cone) {
        Limelight limelight = Limelight.getInstance();

        addCommands(
                new ConditionalCommand(
                        new InstantCommand(limelight::setTapeTopPipeline),
                        new InstantCommand(limelight::setAprilTagsPipeline),
                        cone
                ),

                new SetArmsPositionAngular(() -> ArmConstants.UPPER_CONE_SCORING)
                        .alongWith(new ConditionalCommand(
                                new AdjustToTarget(Math.toRadians(10.07), Math.toRadians(16.62 + 10.07)),
                                new AdjustToAprilTag(xSupplier),
                                cone
                        ))
        );
    }
}
