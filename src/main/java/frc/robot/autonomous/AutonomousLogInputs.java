package frc.robot.autonomous;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AutonomousLogInputs implements LoggableInputs {
//    public Trajectory.State initialPose = new Trajectory.State();
//    public Trajectory.State finalPose = new Trajectory.State();
//    public Pose2d desiredState = new Pose2d();
//    public ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
//    public Rotation2d heading = new Rotation2d();
//    public double time = 0;

    @Override
    public void toLog(LogTable table) {
//        table.put("initialPose", Utils.pose2dToArray(initialPose.poseMeters));
//        table.put("finalPose", Utils.pose2dToArray(finalPose.poseMeters));
//        table.put("desiredState", Utils.pose2dToArray(desiredState));
//        table.put("timeToEnd", finalPose.timeSeconds - time);
//        table.put("desiredSpeeds", Utils.chassisSpeedsToArray(desiredSpeeds));
//        table.put("heading", heading.getDegrees());
    }

    @Override
    public void fromLog(LogTable table) {
    }
}
