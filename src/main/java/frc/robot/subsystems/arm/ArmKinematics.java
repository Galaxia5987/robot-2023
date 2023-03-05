package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.math.AngleUtil;

/**
 * This class contains the kinematics for the arm.
 * All the calculations are carefully documented in the following page:
 * https://www.societyofrobots.com/robot_arm_tutorial.shtml#inverse_kinematics
 * There is a table of contents containing the kinematics section, and the inverse kinematics section.
 */
public class ArmKinematics {
    private final double l1;
    private final double l2;

    public ArmKinematics(double length1, double length2) {
        this.l1 = length1;
        this.l2 = length2;
    }

    /**
     * Forward kinematics for the arm.
     *
     * @param shoulderAngle the angle of the shoulder joint. [rad]
     * @param elbowAngle    the angle of the elbow joint. [rad]
     * @return the position of the end effector. ([m, [m])
     */
    public Translation2d forwardKinematics(double shoulderAngle, double elbowAngle) {
        double x = l1 * Math.cos(shoulderAngle) + l2 * Math.cos(elbowAngle);
        double y = l1 * Math.sin(shoulderAngle) + l2 * Math.sin(elbowAngle);
        return new Translation2d(x, y);
    }

    /**
     * Inverse kinematics for the arm.
     *
     * @param translation the translation of the end effector. [m]
     * @return the angles of the shoulder and elbow joints. ([rad, rad])
     */
    public InverseKinematicsSolution inverseKinematics(Translation2d translation) {
        double x = translation.getX(), y = translation.getY(), s2, c2, K1, K2, psi, theta;
        if (x > 0) {
            y = -y;
        }

        c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        s2 = Math.sqrt(1 - c2 * c2);
        K1 = l1 + l2 * c2;
        K2 = l2 * s2;
        psi = Math.atan2(y, x) - Math.atan2(K2, K1);
        theta = Math.atan2(s2, c2);
        theta = theta + psi;

        if (x > 0) {
            psi = -psi;
            theta = -theta;
        }

        return new InverseKinematicsSolution(psi, theta - psi - Math.PI);
    }

    public InverseKinematicsSolution mirror(InverseKinematicsSolution solution) {
        var position = forwardKinematics(solution.shoulderAngle, solution.elbowAngle);
        double positionAngle = Math.atan2(position.getY(), position.getX());
        return new InverseKinematicsSolution(positionAngle - solution.shoulderAngle, -solution.elbowAngle);
    }

    public InverseKinematicsSolution getVelocities(Translation2d currentPosition, Translation2d desiredVelocity) {
        double x = currentPosition.getX();
        double y = currentPosition.getY();
        double vx = desiredVelocity.getX();
        double vy = desiredVelocity.getY();
        double k = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1) + l1;
        double v = 2 * x * vx + 2 * y * vy;
        double psiDot = (k * k * v / (2 * l1 * Math.pow(1 - k * k, 1.5)) +
                v / (2 * l1 * Math.sqrt(1 - k * k))) /
                ((k * k) / (1 - k * k) + 1);
        double thetaDot = v / Math.sqrt((l2 * l2 - x * x - y * y + l1 * l1 + 2) * (-l2 * l2 + x * x + y * y - l1 * l1 + 2));

        return new InverseKinematicsSolution(psiDot, thetaDot);
    }

    /**
     * A class representing the solution of the inverse kinematics.
     */
    public static class InverseKinematicsSolution {
        public double shoulderAngle;
        public double elbowAngle;

        public InverseKinematicsSolution(double shoulderAngle, double elbowAngle) {
            this.shoulderAngle = AngleUtil.normalize(Rotation2d.fromRadians(shoulderAngle)).getRadians();
            this.elbowAngle = AngleUtil.normalize(Rotation2d.fromRadians(elbowAngle)).getRadians();
        }
    }
}
