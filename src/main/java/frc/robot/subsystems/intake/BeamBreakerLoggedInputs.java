package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class BeamBreakerLoggedInputs implements LoggableInputs {
    public boolean beamBreakerState = true;

    /**
     * Implement the variables into the logger.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("beamBreakerState", beamBreakerState);
    }

    /**
     * Update the variables value from the logger.
     */
    @Override
    public void fromLog(LogTable table) {
        beamBreakerState = table.getBoolean("beamBreakerState", beamBreakerState);
    }
}
