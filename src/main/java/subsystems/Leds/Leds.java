package subsystems.Leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;



public class Leds extends SubsystemBase {
   public static Leds INSTANCE;

    public static Leds getINSTANCE() {
        if(INSTANCE == null){
            return INSTANCE;
        }
        return null;
    }

    public AddressableLED leds = new AddressableLED(0);
   public AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);

   public Leds(){
       leds.setLength(ledBuffer.getLength());
   }
   public void setYellow(){
        for(int i = 0; i<ledBuffer.getLength(); i++ ){
            ledBuffer.setRGB(i,254, 202, 82);
        }
    }
    public void setPurple(){
        for(int i = 0; i<ledBuffer.getLength(); i++ ){
            ledBuffer.setRGB(i,51, 1, 176);
        }
    }

}
