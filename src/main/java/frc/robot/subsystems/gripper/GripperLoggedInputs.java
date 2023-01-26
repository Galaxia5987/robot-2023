package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperLoggedInputs implements LoggableInputs {
    public boolean firstSolenoidState;
    public boolean secondSolenoidState;

    /**
     * puts parameters into the table
     */
    @Override
    public void toLog(LogTable table) {
        table.put("leftSolenoid", firstSolenoidState);
        table.put("rightSolenoid", secondSolenoidState);

    }

    /**
     * extracts the value of the parameters from the logger
     */
    @Override
    public void fromLog(LogTable table) {
        firstSolenoidState = table.getBoolean("leftSolenoid", firstSolenoidState);
        secondSolenoidState = table.getBoolean("rightSolenoid", secondSolenoidState);
    }
}
