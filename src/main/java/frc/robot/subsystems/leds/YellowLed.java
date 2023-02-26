package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class YellowLed extends InstantCommand {

    public YellowLed() {
        super(Leds.getInstance()::setYellow);
    }

}
