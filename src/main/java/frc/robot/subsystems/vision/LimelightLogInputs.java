package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightLogInputs implements LoggableInputs {
    public double lowTargetDistance = 0;
    public double highTargetDistance = 0;
    public double yaw = 0;
    public boolean hasTargets = false;
    public float tagId = 0;
    public Pose2d aprilTagTarget = new Pose2d();

    public LimelightLogInputs() {
    }

    public void toLog(LogTable table) {
        table.put("lowTargetDistance", lowTargetDistance);
        table.put("highTargetDistance", highTargetDistance);
        table.put("yaw", yaw);
        table.put("hasTargets", hasTargets);
        table.put("tagId", tagId);
        table.put("aprilTagCurrentTranslation", new double[]{aprilTagTarget.getX(), aprilTagTarget.getY(), 0});
    }

    public void fromLog(LogTable table) {
        lowTargetDistance = table.getDouble("lowTargetDistance", lowTargetDistance);
        highTargetDistance = table.getDouble("highTargetDistance", highTargetDistance);
        yaw = table.getDouble("yaw", yaw);
        hasTargets = table.getBoolean("hasTargets", hasTargets);
        tagId = table.getFloat("tagId", tagId);
    }
}
