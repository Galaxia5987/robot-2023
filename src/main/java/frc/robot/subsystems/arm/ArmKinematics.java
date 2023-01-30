package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class contains the kinematics for the arm.
 * All the calculations are carefully documented in the following page:
 * https://www.societyofrobots.com/robot_arm_tutorial.shtml#inverse_kinematics
 * There is a table of contents containing the kinematics section, and the inverse kinematics section.
 */
public class ArmKinematics {
    private final double length1;
    private final double length2;

    public ArmKinematics(double length1, double length2) {
        this.length1 = length1;
        this.length2 = length2;
    }

    /**
     * Forward kinematics for the arm.
     *
     * @param shoulderAngle the angle of the shoulder joint. [rad]
     * @param elbowAngle    the angle of the elbow joint. [rad]
     * @return the position of the end effector. ([m, [m])
     */
    public Translation2d forwardKinematics(double shoulderAngle, double elbowAngle) {
        double theta = elbowAngle + shoulderAngle - Math.PI / 2;
        double x = length1 * Math.cos(shoulderAngle) + length2 * Math.sin(theta);
        double y = length1 * Math.sin(shoulderAngle) + length2 * Math.cos(theta);
        return new Translation2d(x, y);
    }

    /**
     * Inverse kinematics for the arm.
     *
     * @param translation the translation of the end effector. [m]
     * @return the angles of the shoulder and elbow joints. ([rad, rad])
     */
    public InverseKinematicsSolution inverseKinematics(Translation2d translation) {
        double x = translation.getX();
        double y = translation.getY();
        double c2 = (x * x + y * y - length1 * length1 - length2 * length2) / (2 * length1 * length2);
        double s2 = Math.sqrt(1 - c2 * c2);
        double shoulderAngle = Math.acos(c2);
        double theta = Math.acos((y * (length1 + length2 * c2) - x * length2 * s2) / (x * x + y * y));
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
