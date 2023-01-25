package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperLoggedInputs implements LoggableInputs {
    public boolean outerSolenoidState;
    public boolean innerSolenoidState;

    /**
     * puts parameters into the table
     */
    @Override
    public void toLog(LogTable table) {
        table.put("leftSolenoid", outerSolenoidState);
        table.put("rightSolenoid", innerSolenoidState);

    }

    /**
     * extracts the value of the parameters from the logger
     */
    @Override
    public void fromLog(LogTable table) {
        outerSolenoidState = table.getBoolean("leftSolenoid", outerSolenoidState);
        innerSolenoidState = table.getBoolean("rightSolenoid", innerSolenoidState);
    }
}
