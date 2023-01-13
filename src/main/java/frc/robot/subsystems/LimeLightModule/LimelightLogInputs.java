package frc.robot.subsystems.LimeLightModule;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
public class LimelightLogInputs implements LoggableInputs {
    public double s;
    public long id;
    double x;
    double y;
    double a;
    boolean v;
    long pipeLine;

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
        table.get("x");
        table.get("y");
        table.get("a");
        table.get("v");
        table.get("pipeLine");
    }
}
