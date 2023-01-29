package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

public class AllianceFlipUtil {
    /**
     * Flips a translation to the correct side of the field based on the current alliance color.
     */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(VisionConstants.FIELD_LENGTH - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    /**
     * Flips a rotation based on the current alliance color.
     */
    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance color.
     */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                    VisionConstants.FIELD_LENGTH - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }

    /**
     * Flips a trajectory state to the correct side of the field based on the current alliance color.
     */
    public static Trajectory.State apply(Trajectory.State state) {
        if (shouldFlip()) {
            return new Trajectory.State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(
                            VisionConstants.FIELD_LENGTH - state.poseMeters.getX(),
                            state.poseMeters.getY(),
                            new Rotation2d(
                                    -state.poseMeters.getRotation().getCos(),
                                    state.poseMeters.getRotation().getSin())),
                    -state.curvatureRadPerMeter);
        } else {
            return state;
        }
    }

    private static boolean shouldFlip() {
        return DriverStation.getAlliance() == Alliance.Red;
    }
}