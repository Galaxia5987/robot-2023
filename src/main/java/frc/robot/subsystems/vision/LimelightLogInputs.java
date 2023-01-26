package frc.robot.subsystems.vision;

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
    public Limelight.AprilTagTarget aprilTagTarget = new Limelight.AprilTagTarget(
            new Translation2d(),
            new Translation2d(),
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d()
    );

    public LimelightLogInputs() {
    }

    public void toLog(LogTable table) {
        table.put("targetDistance", targetDistance);
        table.put("yaw", yaw);
        table.put("hasTargets", hasTargets);
        table.put("tagId", tagId);
        table.put("aprilTagDesiredTranslation", new double[]{aprilTagTarget.desiredTranslation.getX(), aprilTagTarget.desiredTranslation.getY(), aprilTagTarget.targetYaw.getDegrees()});
        table.put("aprilTagCurrentTranslation", new double[]{aprilTagTarget.currentTranslation.getX(), aprilTagTarget.currentTranslation.getY(), aprilTagTarget.targetYaw.getDegrees()});
    }

    public void fromLog(LogTable table) {
        targetDistance = table.getDouble("targetDistance", targetDistance);
        yaw = table.getDouble("yaw", yaw);
        hasTargets = table.getBoolean("hasTargets", hasTargets);
        tagId = table.getFloat("tagId", tagId);
    }
}
