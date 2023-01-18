package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class contains the kinematics for the arm.
 * All the calculations are carefully documented in the following page:
 * https://www.societyofrobots.com/robot_arm_tutorial.shtml#inverse_kinematics
 * There is a table of contents containing the kinematics section, and the inverse kinematics section.
 */
public class ArmKinematics {
    private final double l1;
    private final double l2;

    public ArmKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /**
     * Forward kinematics for the arm.
     *
     * @param shoulderAngle the angle of the shoulder joint. [rad]
     * @param elbowAngle the angle of the elbow joint. [rad]
     * @return the position of the end effector. ([m, [m])
     */
    public Translation2d forwardKinematics(double shoulderAngle, double elbowAngle) {
        double theta = elbowAngle + shoulderAngle - Math.PI / 2;
        double x = l1 * Math.cos(shoulderAngle) + l2 * Math.sin(theta);
        double y = l1 * Math.sin(shoulderAngle) + l2 * Math.cos(theta);
        return new Translation2d(x, y);
    }

    /**
     * Inverse kinematics for the arm.
     *
     * @param x the x coordinate of the end effector. [m]
     * @param y the y coordinate of the end effector. [m]
     * @return the angles of the shoulder and elbow joints. ([rad, rad])
     */
    public InverseKinematicsSolution inverseKinematics(double x, double y) {
        double c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        double s2 = Math.sqrt(1 - c2 * c2);
        double shoulderAngle = Math.acos(c2);
        double theta = Math.acos((y * (l1 + l2 * c2) - x * l2 * s2) / (x * x + y * y));
        double elbowAngle = theta - shoulderAngle + Math.PI / 2;

        return new InverseKinematicsSolution(shoulderAngle, elbowAngle);
    }

    /**
     * A class representing the solution of the inverse kinematics.
     */
    public static class InverseKinematicsSolution {
        public double shoulderAngle;
        public double elbowAngle;

        public InverseKinematicsSolution(double shoulderAngle, double elbowAngle) {
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }
    }
}
