package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

//@AutoLog
//public class LimelightLogInputs {
//    public double targetDistance = 0;
//    public double yaw = 0;
//    public boolean hasTargets = false;
//    public float tagId = 0;
//}

public class LimelightLogInputs implements LoggableInputs {
    public double targetDistance = 0;
    public double yaw = 0;
    public boolean hasTargets = false;
    public float tagId = 0;
    public Pose3d aprilTagTarget = new Pose3d();

    public LimelightLogInputs() {
    }

    public void toLog(LogTable table) {
        table.put("targetDistance", targetDistance);
        table.put("yaw", yaw);
        table.put("hasTargets", hasTargets);
        table.put("tagId", tagId);
        table.put("aprilTagCurrentTranslation", new double[]{aprilTagTarget.getX(), aprilTagTarget.getY(), 0});
    }

    public void fromLog(LogTable table) {
        targetDistance = table.getDouble("targetDistance", targetDistance);
        yaw = table.getDouble("yaw", yaw);
        hasTargets = table.getBoolean("hasTargets", hasTargets);
        tagId = table.getFloat("tagId", tagId);
    }
}
