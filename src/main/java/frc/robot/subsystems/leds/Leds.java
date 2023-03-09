package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Leds extends SubsystemBase {
    public static Leds INSTANCE;
    public AddressableLED leds = new AddressableLED(Ports.Leds.LED);
    public AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);

    private Color color = Color.kPurple;

    private Leds() {
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }

    public static Leds getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Leds();
        }
        return INSTANCE;
    }

    public void setYellow() {
        color = Color.kYellow;
    }

    public void setPurple() {
        color = Color.kPurple;
    }

    public void toggle() {
        if (color == Color.kYellow) {
            color = Color.kPurple;
        } else {
            color = Color.kYellow;
        }
    }

    public boolean inConeMode() {
        return color == Color.kYellow;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        leds.setData(ledBuffer);
    }
}
