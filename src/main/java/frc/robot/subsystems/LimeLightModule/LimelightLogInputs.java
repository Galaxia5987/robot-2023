package frc.robot.subsystems.LimeLightModule;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightLogInputs implements LoggableInputs {
    double x;
    double y;
    double a;
    boolean v;
    double pipeLine;

    @Override
    public void toLog(LogTable table) {
        table.put("x", x);
        table.put("y", y);
        table.put("a", a);
        table.put("v", v);
        table.put("pipeLine", pipeLine);
    }

    @Override
    public void fromLog(LogTable table) {

    }
}
