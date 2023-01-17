package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmKinematics {
    private final double l1;
    private final double l2;

    public ArmKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    public Translation2d forwardKinematics(double theta1, double theta2) {
        double x = l1 * Math.cos(theta1) + l2 * Math.cos(theta1 + theta2);
        double y = l1 * Math.sin(theta1) + l2 * Math.sin(theta1 + theta2);
        return new Translation2d(x, y);
    }

    public InverseKinematicsSolution inverseKinematics(double x, double y) {
        double c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        double s2 = Math.sqrt(1 - c2 * c2);
        double theta1 = Math.acos(c2);
        double theta2 = Math.acos((y * (l1 + l2 * c2) - x * l2 * s2) / (x * x + y * y));

        return new InverseKinematicsSolution(theta1, theta2);
    }

    public record InverseKinematicsSolution(double theta1, double theta2) {
    }
}
