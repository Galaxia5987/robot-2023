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
    private Color secondaryColor = new Color(0, 0, 0);
    private double blinkTime = LedConstants.MID_BLINK_TIME;
    private boolean blink = false;
    private boolean isRainbow = false;
    private int rainbowHue = 0;

    private Mode mode = Mode.PRIMARY_COLOR;

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

    public void setColor(Color color) {
        this.color = color;
    }

    public void setBlinkTime(double blinkTime) {
        this.blinkTime = blinkTime;
    }

    public void setBlink(boolean blink) {
        this.blink = blink;
    }

    public void setBlink(boolean blink, Color secondaryColor) {
        this.blink = blink;
        this.secondaryColor = secondaryColor;
    }

    public void setBlink(boolean blink, Color secondaryColor, double blinkTime) {
        this.blink = blink;
        this.secondaryColor = secondaryColor;
        this.blinkTime = blinkTime;
    }

    public void toggleRainbow() {
        if (!isRainbow) {
            this.mode = Mode.ELSE;
            this.isRainbow = true;
        } else {
            this.mode = Mode.PRIMARY_COLOR;
            this.isRainbow = false;
        }
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
                if (mode == Mode.PRIMARY_COLOR) {
                    mode = Mode.SECONDARY_COLOR;
                } else {
                    mode = Mode.PRIMARY_COLOR;
                }
                timer.reset();
            }
        } else if(mode != Mode.ELSE){
            mode = Mode.PRIMARY_COLOR;
        }


        if (mode == Mode.PRIMARY_COLOR) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, color);
            }
            leds.setData(ledBuffer);
        } else if (mode == Mode.SECONDARY_COLOR) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, secondaryColor);
            }
            leds.setData(ledBuffer);
        }
        if (isRainbow){
            mode = Mode.ELSE;
            System.out.println("before loop statement");
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, rainbowHue, 255, 180);
                rainbowHue += (180 / ledBuffer.getLength());
                leds.setData(ledBuffer);
                System.out.println("in-loop check statement");
            rainbowHue%=180;
            }
            System.out.println("after loop statement");
        }
    }

    private enum Mode {
        SECONDARY_COLOR, PRIMARY_COLOR, ELSE
    }
}
