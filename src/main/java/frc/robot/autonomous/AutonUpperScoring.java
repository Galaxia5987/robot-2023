package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandgroups.UpperScoring;
import frc.robot.subsystems.leds.PurpleLed;
import frc.robot.subsystems.leds.YellowLed;

public class AutonUpperScoring extends SequentialCommandGroup {

    public AutonUpperScoring(boolean isCone) {
        addCommands(
                isCone ? new YellowLed() :
                        new PurpleLed(),
                isCone ? new UpperScoring().withTimeout(2.8) :
                        new UpperScoring().withTimeout(2)
        );
    }
}