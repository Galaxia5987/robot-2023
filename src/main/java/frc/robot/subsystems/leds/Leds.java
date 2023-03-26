package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Leds extends SubsystemBase {
    public static Leds INSTANCE;
    private final Timer timer = new Timer();
    public AddressableLED leds = new AddressableLED(Ports.Leds.LED);
    public AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
    private Color color = Color.kPurple;
    private double blinkTime = 0.25;
    private boolean blink = false;

    private Mode mode = Mode.ON;

    private Leds() {
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();

        timer.start();
        timer.reset();
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

    public void setBlinkTime(double blinkTime) {
        this.blinkTime = blinkTime;
    }

    public void setBlink(boolean blink) {
        this.blink = blink;
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
        if (blink) {
            if (timer.hasElapsed(blinkTime)) {
                if (mode == Mode.ON) {
                    mode = Mode.OFF;
                } else {
                    mode = Mode.ON;
                }
                timer.reset();
            }
        } else {
            mode = Mode.ON;
        }

        if (mode == Mode.ON) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, color);
            }
            leds.setData(ledBuffer);
        } else {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kBlack);
            }
            leds.setData(ledBuffer);
        }
    }

    private enum Mode {
        OFF, ON
    }
}
