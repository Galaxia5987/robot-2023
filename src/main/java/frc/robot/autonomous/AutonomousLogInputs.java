package frc.robot.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.utils.Utils;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AutonomousLogInputs implements LoggableInputs {
    public Trajectory.State initialPose = new Trajectory.State();
    public Trajectory.State finalPose = new Trajectory.State();
    public Trajectory.State desiredState = new Trajectory.State();
    public ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    public double time = 0;

    @Override
    public void toLog(LogTable table) {
        table.put("initialPose", Utils.pose2dToArray(initialPose.poseMeters));
        table.put("finalPose", Utils.pose2dToArray(finalPose.poseMeters));
        table.put("desiredState", Utils.pose2dToArray(desiredState.poseMeters));
        table.put("timeToEnd", finalPose.timeSeconds - time);
        table.put("desiredSpeeds", Utils.chassisSpeedsToArray(desiredSpeeds));
    }

    @Override
    public void fromLog(LogTable table) {
    }
}