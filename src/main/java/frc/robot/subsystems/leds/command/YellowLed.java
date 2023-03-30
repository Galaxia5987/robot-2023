package frc.robot.subsystems.leds.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.leds.Leds;

public class YellowLed extends InstantCommand {

    public YellowLed() {
        super(Leds.getInstance()::setYellow);
    }

}
