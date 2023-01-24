package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    double supMAX;
    double infMAX;
    boolean solenoid;

    @Override
    public void toLog(LogTable table) {
        table.put("superiorMAX", supMAX);
        table.put("inferiorMAX", infMAX);
        table.put("solenoid", solenoid);
    }

    @Override
    public void fromLog(LogTable table) {
        supMAX = table.getDouble("superiorMAX", 0);
        infMAX = table.getDouble("inferiorMAX", 0);
        solenoid = table.getBoolean("solenoid", false);
    }
}
