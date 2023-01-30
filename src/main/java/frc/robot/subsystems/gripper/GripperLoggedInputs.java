package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperLoggedInputs implements LoggableInputs {
    public boolean isOpen;

    /**
     * Puts parameters into the table.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("gripperSolenoid", isOpen);
    }

    /**
     * Extracts the value of the parameters from the logger.
     */
    @Override
    public void fromLog(LogTable table) {
        isOpen = table.getBoolean("gripperSolenoid", isOpen);
    }
}
