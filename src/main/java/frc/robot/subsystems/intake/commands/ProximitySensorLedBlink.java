package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.ProximitySensor;
import frc.robot.subsystems.leds.Leds;


public class ProximitySensorLedBlink extends CommandBase {
    private final ProximitySensor proximitySensor = ProximitySensor.getInstance();
    private final Leds leds = Leds.getInstance();

    private boolean ProximitySensorState = proximitySensor.isBeamBlocked();

    public ProximitySensorLedBlink() {
        this.ProximitySensorState = proximitySensor.isBeamBlocked();
        addRequirements(proximitySensor);
    }

    @Override
    public void execute() {
        Color color = leds.ledBuffer.getLED(0);
        while (ProximitySensorState){
            for (int i=0; i<leds.ledBuffer.getLength(); i++)
                leds.ledBuffer.setLED(i, new Color(0, 0, 0));
            for (int i=0; i<leds.ledBuffer.getLength(); i++)
                leds.ledBuffer.setLED(i, color);
        }
    }
}
