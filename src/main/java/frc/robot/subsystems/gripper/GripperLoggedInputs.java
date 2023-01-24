package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperLoggedInputs implements LoggableInputs {
    boolean leftSolenoidState;
    boolean rightSolenoidState;

    /**
     * puts parameters into the table
     */
    @Override
    public void toLog(LogTable table) {
        table.put("leftSolenoid", leftSolenoidState);
        table.put("rightSolenoid", rightSolenoidState);

    }

    /**
     * extracts the value of the parameters from the logger
     */
    @Override
    public void fromLog(LogTable table) {
        leftSolenoidState = table.getBoolean("leftSolenoid", leftSolenoidState);
        rightSolenoidState = table.getBoolean("rightSolenoid", rightSolenoidState);
    }
}
