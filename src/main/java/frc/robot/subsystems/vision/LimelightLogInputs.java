package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
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
    public Pose2d botPose = new Pose2d();

    public LimelightLogInputs() {
    }

    public void toLog(LogTable table) {
        table.put("targetDistance", targetDistance);
        table.put("yaw", yaw);
        table.put("hasTargets", hasTargets);
        table.put("tagId", tagId);
        table.put("botPose", new double[] {botPose.getX(), botPose.getY(), botPose.getRotation().getDegrees()});
    }

    public void fromLog(LogTable table) {
        targetDistance = table.getDouble("targetDistance", targetDistance);
        yaw = table.getDouble("yaw", yaw);
        hasTargets = table.getBoolean("hasTargets", hasTargets);
        tagId = table.getFloat("tagId", tagId);
    }
}
