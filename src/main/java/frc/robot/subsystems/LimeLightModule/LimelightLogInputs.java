package frc.robot.subsystems.LimeLightModule;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
public class LimelightLogInputs implements LoggableInputs {
    Limelight limelight = new Limelight();

    public double x = limelight.getHorizantalOffset();
    public double y = limelight.getVerticalOffset();
//    public double a;
//    public double s;
    public long id = limelight.getId();
    boolean v = limelight.hasTarget();

    double distance = limelight.getDistance();
    long pipeLine = limelight.getPipeline();

    @Override
    public void toLog(LogTable table) {
        table.put("id", id);
        table.put("v", v);
        table.put("distance", distance);
        table.put("pipeLine", pipeLine);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("id");
        table.get("v");
        table.get("distance");
        table.get("pipeLine");
    }
}
