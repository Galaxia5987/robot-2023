package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;


public class BeamBreaker extends LoggedSubsystem<BeamBreakerLoggedInputs> {
    public DigitalInput beam = new DigitalInput(Ports.Intake.BEAM_BREAKER_SENSOR);
    boolean isBlocking;

    public BeamBreaker(BeamBreakerLoggedInputs inputs) {
        super(inputs);
    }


    public boolean isBeamBlocked() {
        if(beam.get()== false){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void updateInputs() {
        loggerInputs.beamBreakerState = isBeamBlocked();
    }

    @Override
    public String getSubsystemName() {
        return "BeamBreaker";
    }
}
