package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.leds.PurpleLed;
import frc.robot.subsystems.leds.YellowLed;

public class AutonUpperScoring extends SequentialCommandGroup {

    public AutonUpperScoring(boolean isCone) {
        addCommands(
                isCone ? new YellowLed() :
                        new PurpleLed(),
                isCone ? new UpperScoring().withTimeout(1.7) :
                        new UpperScoring().withTimeout(1)
        );
    }
}
