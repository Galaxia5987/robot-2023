package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Leds extends SubsystemBase {
    private static Leds INSTANCE;
    private final AddressableLED leds = new AddressableLED(Ports.Leds.LED);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);

    private Color color;
    private double blinkTime = 0;

    private final Timer timer = new Timer();

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

    public void setColor(Color color) {
        this.color = color;
    }

    public void setBlinkTime(double blinkTime) {
        this.blinkTime = blinkTime;
    }

    public void set(Color color, double blinkTime) {
        setColor(color);
        setBlinkTime(blinkTime);
    }

    @Override
    public void periodic() {
        if (timer.hasElapsed(blinkTime)) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, color);
            }
            timer.reset();
        } else if (timer.hasElapsed(blinkTime / 2)) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kBlack);
            }
        }

    }
}
