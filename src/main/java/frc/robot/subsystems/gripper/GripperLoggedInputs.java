package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperLoggedInputs implements LoggableInputs {
    public boolean solenoidState;

    /**
     * Puts parameters into the table.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("solenoid", solenoidState);
    }

    /**
     * Extracts the value of the parameters from the logger.
     */
    @Override
    public void fromLog(LogTable table) {
        solenoidState = table.getBoolean("solenoid", solenoidState);
    }
}
