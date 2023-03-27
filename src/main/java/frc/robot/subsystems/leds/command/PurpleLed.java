package frc.robot.subsystems.leds.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.leds.Leds;

public class PurpleLed extends InstantCommand {

    public PurpleLed() {
        super(Leds.getInstance()::setPurple);
    }

}
