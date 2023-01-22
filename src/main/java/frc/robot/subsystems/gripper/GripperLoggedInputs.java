package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperLoggedInputs implements LoggableInputs {
    boolean leftSolenoid;
    boolean rightSolenoid;

    /**
     * puts parameters into the table
     */
    @Override
    public void toLog(LogTable table) {
        table.put("leftSolenoid", leftSolenoid);
        table.put("rightSolenoid", rightSolenoid);

    }

    /**
     * extracts the value of the parameters from the logger
     */
    @Override
    public void fromLog(LogTable table) {
        leftSolenoid = table.getBoolean("leftSolenoid", leftSolenoid);
        rightSolenoid = table.getBoolean("rightSolenoid", rightSolenoid);
    }
}
