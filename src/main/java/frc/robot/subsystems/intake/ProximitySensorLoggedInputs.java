package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ProximitySensorLoggedInputs implements LoggableInputs {
    public boolean ProximitySensorState = true;

    /**
     * Implement the variables into the logger.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("ProximitySensorState", ProximitySensorState);
    }

    /**
     * Update the variables value from the logger.
     */
    @Override
    public void fromLog(LogTable table) {
        ProximitySensorState = table.getBoolean("ProximitySensorState", ProximitySensorState);
    }
}
