package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PurpleLed extends CommandBase {
    private final  Leds leds = Leds.getInstance();

    public PurpleLed() {
        addRequirements(leds);
    }

    @Override
    public void execute() {
        leds.setPurple();
    }

}
