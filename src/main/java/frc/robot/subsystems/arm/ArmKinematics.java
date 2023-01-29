package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class contains the kinematics for the arm.
 */
public class ArmKinematics {
    private final double shoulderLength;
    private final double elbowLength;

    public ArmKinematics(double shoulderLength, double elbowLength) {
        this.shoulderLength = shoulderLength;
        this.elbowLength = elbowLength;
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
        double x = shoulderLength * Math.cos(shoulderAngle) + elbowLength * Math.sin(theta);
        double y = shoulderLength * Math.sin(shoulderAngle) + elbowLength * Math.cos(theta);
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
        if (x > 0) {
            y = -y;
        }
        double c2 = (x * x + y * y - shoulderLength * shoulderLength - elbowLength * elbowLength) / (2 * shoulderLength * elbowLength);
        double s2 = Math.sqrt(1 - c2 * c2);
        double k1 = shoulderLength + elbowLength * c2;
        double k2 = elbowLength * s2;
        double psi = Math.atan2(y, x) - Math.atan2(k2, k1);
        double theta = Math.atan2(s2, c2);
        theta = theta + psi;
        if (x > 0) {
            psi = -psi;
            theta = -theta;
        }
        return new InverseKinematicsSolution(psi, theta);
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
