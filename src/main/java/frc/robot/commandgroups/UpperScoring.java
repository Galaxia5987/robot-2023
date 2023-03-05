package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.arm.commands.SetElbowAngle;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Limelight;

public class UpperScoring extends SequentialCommandGroup {

    public UpperScoring() {
        Arm arm = Arm.getInstance();
        addCommands(
                new ConditionalCommand(
                        new SetArmsPositionAngular(() -> ArmConstants.UPPER_CONE_SCORING1, 0.1)
                                .andThen(new WaitCommand(0.3)),
                        new InstantCommand(),
                        Leds.getInstance()::inConeMode
                ),

                new ConditionalCommand(
                        new SetElbowAngle(178)
                                .raceWith(new RunCommand(() -> arm.setShoulderJointPower(-0.05))
                                        .withTimeout(1.2)),
                        new SetArmsPositionAngular(() -> ArmConstants.UPPER_CUBE_SCORING),
                        Leds.getInstance()::inConeMode
                )
        );
    }
}
