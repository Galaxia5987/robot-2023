package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Limelight;

public class UpperScoring extends SequentialCommandGroup {

    public UpperScoring() {
        addCommands(
                new ConditionalCommand(
                        new SetArmsPositionAngular(() -> ArmConstants.UPPER_CONE_SCORING1, 0.05)
                                .andThen(new WaitCommand(0.3)),
                        new InstantCommand(),
                        Leds.getInstance()::inConeMode
                ),

                new SetArmsPositionAngular(() -> ArmConstants.UPPER_CONE_SCORING2)
        );
    }
}
