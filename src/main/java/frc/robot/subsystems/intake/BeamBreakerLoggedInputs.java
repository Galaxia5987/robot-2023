package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class BeamBreakerLoggedInputs implements LoggableInputs {
    public boolean beamBreakerState;

    /**
     * implement the variables into the logger
     */
    @Override
    public void toLog(LogTable table) {
        table.put("beamBreakerState", beamBreakerState);
    }

    /**
     * update the variables value from the logger
     */
    @Override
    public void fromLog(LogTable table) {
        beamBreakerState = table.getBoolean("beamBreakerState", beamBreakerState);
    }
}
