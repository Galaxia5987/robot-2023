package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.leds.Leds;

public class UpperScoring extends SequentialCommandGroup {

    public UpperScoring() {
        Arm arm = Arm.getInstance();
        addCommands(
                new ConditionalCommand(
                        new SetArmsPositionAngular(() -> ArmConstants.UPPER_CONE_SCORING, 0.1),
                        new SetArmsPositionAngular(() -> ArmConstants.UPPER_CUBE_SCORING),
                        Leds.getInstance()::inConeMode
                )
        );
    }
}
