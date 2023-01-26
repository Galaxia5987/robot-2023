package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    public double motorPower;
    public boolean leftSolenoidState;
    public boolean rightSolenoidState;

    /**
     * implement the variables inside the table
     */
    @Override
    public void toLog(LogTable table) {
        table.put("superiorMAX", motorPower);
        table.put("left solenoid", leftSolenoidState);
        table.put("right solenoid", rightSolenoidState);
    }

    @Override
    public void fromLog(LogTable table) {
        motorPower = table.getDouble("motor power", motorPower);
        leftSolenoidState = table.getBoolean("left solenoid", leftSolenoidState);
        rightSolenoidState = table.getBoolean("right solenoid", rightSolenoidState);
    }
}
