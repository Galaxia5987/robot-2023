package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PurpleLed extends InstantCommand {

    public PurpleLed() {
        super(Leds.getInstance()::setPurple);
    }

}
