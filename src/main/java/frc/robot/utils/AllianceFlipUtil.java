package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

public class AllianceFlipUtil {
    /**
     * Flips a translation to the correct side of the field based on the current alliance color.
     */
    public static Translation2d apply(Alliance alliance, Translation2d translation) {
        if (shouldFlip(alliance)) {
            return new Translation2d(VisionConstants.FIELD_LENGTH - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    /**
     * Flips a rotation based on the current alliance color.
     */
    public static Rotation2d apply(Alliance alliance, Rotation2d rotation) {
        if (shouldFlip(alliance)) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance color.
     */
    public static Pose2d apply(Alliance alliance, Pose2d pose) {
        if (shouldFlip(alliance)) {
            return new Pose2d(
                    VisionConstants.FIELD_LENGTH - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance color.
     */
    public static Pose3d apply(Alliance alliance, Pose3d pose) {
        var pose2d = apply(alliance, pose.toPose2d());
        return new Pose3d(new Translation3d(pose2d.getX(), pose2d.getY(), pose.getZ()), pose.getRotation());
    }

    public static ChassisSpeeds apply(Alliance alliance, ChassisSpeeds speeds) {
        Rotation2d heading = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double magnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Rotation2d flippedHeading = apply(alliance, heading);
        return new ChassisSpeeds(
                flippedHeading.getCos() * magnitude,
                flippedHeading.getSin() * magnitude,
                speeds.omegaRadiansPerSecond);
    }

    /**
     * Flips a trajectory state to the correct side of the field based on the current alliance color.
     */
    public static Trajectory.State apply(Alliance alliance, Trajectory.State state) {
        if (shouldFlip(alliance)) {
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

    private static boolean shouldFlip(Alliance alliance) {
        return alliance == Alliance.Red;
    }
}