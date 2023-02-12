package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;


public class BeamBreaker extends LoggedSubsystem<BeamBreakerLoggedInputs> {
    private static BeamBreaker INSTANCE;
    private final DigitalInput beam = new DigitalInput(Ports.Intake.BEAM_BREAKER_SENSOR);

    private BeamBreaker() {
        super(new BeamBreakerLoggedInputs());
    }

    public static BeamBreaker getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new BeamBreaker();
        }
        return INSTANCE;
    }

    /**
     * @return whether the beam breaker is blocked.
     */

    public boolean isBeamBlocked() {
        return !beam.get();
    }

    /**
     * Update the logger variables.
     */
    @Override
    public void updateInputs() {
        loggerInputs.beamBreakerState = isBeamBlocked();
    }

    @Override
    public String getSubsystemName() {
        return "BeamBreaker";
    }
}
