package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.leds.Leds;


public class BeamBreakerLedBlink extends CommandBase {
    private final BeamBreaker beamBreaker = BeamBreaker.getInstance();
    private final Leds leds = Leds.getInstance();

    private boolean beamBreakerState = beamBreaker.isBeamBlocked();

    public BeamBreakerLedBlink() {
        this.beamBreakerState = beamBreaker.isBeamBlocked();
        addRequirements(beamBreaker);
    }

    @Override
    public void execute() {
        Color color = leds.ledBuffer.getLED(0);
        while (beamBreakerState){
            for (int i=0; i<leds.ledBuffer.getLength(); i++)
                leds.ledBuffer.setLED(i, new Color(0, 0, 0));
            for (int i=0; i<leds.ledBuffer.getLength(); i++)
                leds.ledBuffer.setLED(i, color);
        }
    }
}
