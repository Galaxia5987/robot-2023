// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {
    }

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        Pose2d botPose = new Pose2d(0, 0, new Rotation2d());
        Pose3d aprilTag = new Pose3d(new Translation3d(1, 1, 0), new Rotation3d());
        ChassisSpeeds currVelocity = new ChassisSpeeds(0, 0, 0);
        var pStart = new PathPoint(
                botPose.getTranslation(),
                new Rotation2d(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond),
                botPose.getRotation(),
                Math.hypot(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond));
        var pEnd = new PathPoint(
                aprilTag.getTranslation().toTranslation2d(),
                aprilTag.getRotation().toRotation2d(),
                aprilTag.getRotation().toRotation2d(),
                0);
        var trajectory = PathPlanner.generatePath(new PathConstraints(5, 3), false,
                pStart, pEnd);
//        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.Alliance.Red);
//        RobotBase.startRobot(Robot::new);

        try {
            double time = 0;
            StringBuilder output = new StringBuilder();
            File file = new File("generatedPath.csv");
            file.createNewFile();
            FileWriter writer = new FileWriter(file);

            while (time < trajectory.getTotalTimeSeconds()) {
                var state = trajectory.sample(time);
                output.append(time)
                        .append(",")
                        .append(state.poseMeters.getX())
                        .append(",")
                        .append(state.poseMeters.getY())
                        .append("\n");
                for (char c : output.toString().toCharArray()) {
                    writer.append(c);
                }
                output = new StringBuilder();
                time += 0.02;
            }

            writer.close();
        } catch (Throwable t) {
            t.printStackTrace();
        }

    }
}
