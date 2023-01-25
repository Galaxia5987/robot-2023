package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    public double motorPower;
    public boolean solenoidState;

    /**
     * implement the variables inside the table
     */
    @Override
    public void toLog(LogTable table) {
        table.put("superiorMAX", motorPower);
        table.put("solenoid", solenoidState);
    }

    @Override
    public void fromLog(LogTable table) {
        motorPower = table.getDouble("superiorMAX", motorPower);
        solenoidState = table.getBoolean("solenoid", solenoidState);
    }
}
