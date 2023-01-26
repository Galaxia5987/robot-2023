package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;


public class BeamBreaker extends LoggedSubsystem<BeamBreakerLoggedInputs> {
    public DigitalInput beam = new DigitalInput(Ports.Intake.BEAMBREAKER);
    boolean isBlocking;

    public BeamBreaker(BeamBreakerLoggedInputs inputs) {
        super(inputs);
    }


    public boolean getBeam() {
        return beam.get();
    }

    @Override
    public void updateInputs() {
        loggerInputs.beamBreakerState = getBeam();
    }

    @Override
    public String getSubsystemName() {
        return "BeamBreaker";
    }
}
