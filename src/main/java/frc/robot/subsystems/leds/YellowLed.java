package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class YellowLed extends CommandBase {
    private final Leds leds = Leds.getInstance();


    public YellowLed() {
        addRequirements(leds);
    }

    @Override
    public void execute() {
        leds.setYellow();
    }


}
