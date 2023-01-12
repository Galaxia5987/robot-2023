package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
public class LimelightLogInputs implements LoggableInputs {
    public static LimelightLogInputs INSTANCE = null;
    public double targetDistance;
    public double yaw;
    public boolean hasTargets;
    public float tagId;

    public LimelightLogInputs() {
    }

    /**
     * if there is no instance of the limelightLogInputs class it creates one and returns it
     * @return
     */
    public static LimelightLogInputs getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LimelightLogInputs();
        }
        return INSTANCE;
    }

    public void toLog(LogTable table) {
        table.put("targetDistance", targetDistance);
        table.put("yaw", yaw);
        table.put("hasTargets", hasTargets);
        table.put("tagId", tagId);
    }

    public void fromLog(LogTable table) {
        targetDistance = table.getDouble("targetDistance", targetDistance);
        yaw = table.getDouble("yaw", yaw);
        hasTargets = table.getBoolean("hasTargets", hasTargets);
        tagId = table.getFloat("tagId", tagId);
    }
}
