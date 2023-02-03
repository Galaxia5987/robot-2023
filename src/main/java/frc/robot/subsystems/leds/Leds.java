package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Leds extends SubsystemBase {
    public static Leds INSTANCE;
    private final Color Yellow = new Color(254, 202, 82);
    private final Color Purple = new Color(51, 1, 176);
    public AddressableLED leds = new AddressableLED(Ports.Leds.LED);
    public AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);

    public Leds() {
        leds.setLength(ledBuffer.getLength());
    }

    public static Leds getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Leds();
        }
        return INSTANCE;
    }

    public void setYellow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, Yellow);
        }
    }

    public void setPurple() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, Purple);

        }
    }

}
