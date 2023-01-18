package frc.robot.subsystems.arm;

import frc.robot.utils.motors.FalconConstants;
import org.ejml.data.MatrixType;
import org.ejml.simple.SimpleMatrix;

/**
 * This class is used to calculate the feedforward for the arm.
 * Notice that this class does not calculate velocities or accelerations,
 * and relies on the user to do so (in the subsystem class).
 *
 * Documentation for the math used in this class can be found here:
 * https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060?u=dan
 */
public class ArmSystemModel {
    private final double m1;
    private final double m2;
    private final double l1;
    private final double r1;
    private final double r2;
    private final double I1;
    private final double I2;
    private final double g1;
    private final double g2;

    private final SimpleMatrix M_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);
    private final SimpleMatrix C_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);
    private final SimpleMatrix Tg_VECTOR = new SimpleMatrix(2, 1, MatrixType.DDRM);
    private final SimpleMatrix Kb_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);
    private final SimpleMatrix B_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);

    /**
     * The constructor for the system model.
     *
     * @param systemConstants the system constants of the arm.
     */
    public ArmSystemModel(SystemConstants systemConstants) {
        var shoulderJointConstants = systemConstants.getShoulderJointConstants();
        var elbowJointConstants = systemConstants.getElbowJointConstants();
        this.m1 = shoulderJointConstants.mass;
        this.m2 = elbowJointConstants.mass;
        this.l1 = shoulderJointConstants.length;
        this.r1 = shoulderJointConstants.cmRadius;
        this.r2 = elbowJointConstants.cmRadius;
        this.I1 = shoulderJointConstants.momentOfInertia;
        this.I2 = elbowJointConstants.momentOfInertia;
        this.g1 = shoulderJointConstants.gearing;
        this.g2 = elbowJointConstants.gearing;
        int n1 = shoulderJointConstants.numberOfMotors;
        int n2 = elbowJointConstants.numberOfMotors;

        B_MATRIX.set(0, 0, g1 * n1 * FalconConstants.Kt / FalconConstants.R);
        B_MATRIX.set(1, 1, g2 * n2 * FalconConstants.Kt / FalconConstants.R);
        B_MATRIX.set(1, 0, 0);
        B_MATRIX.set(0, 1, 0);

        Kb_MATRIX.set(0, 0, g1 * g1 * n1 * FalconConstants.Kt / FalconConstants.R / FalconConstants.Kv);
        Kb_MATRIX.set(1, 1, g2 * g2 * n2 * FalconConstants.Kt / FalconConstants.R / FalconConstants.Kv);
        Kb_MATRIX.set(1, 0, 0);
        Kb_MATRIX.set(0, 1, 0);
    }

    /**
     * Calculates the M matrix.
     *
     * @param theta2 the angle of the elbow joint. [rad]
     */
    public void updateMMatrix(double theta2) {
        double c2 = Math.cos(theta2);
        double diagonal = m2 * (r2 * r2 + l1 * r2 * c2) + I2;

        M_MATRIX.set(0, 0, m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2 + 2 * l1 * r2 * c2) + I1 + I2);
        M_MATRIX.set(0, 1, diagonal);
        M_MATRIX.set(1, 0, diagonal);
        M_MATRIX.set(1, 1, m2 * r2 * r2 + I2);
    }

    /**
     * Calculates the C matrix.
     *
     * @param theta2 the angle of the elbow joint. [rad]
     * @param theta1Dot the angular velocity of the shoulder joint. [rad/s]
     * @param theta2Dot the angular velocity of the elbow joint. [rad/s]
     */
    public void updateCMatrix(double theta2, double theta1Dot, double theta2Dot) {
        double s2 = Math.sin(theta2);

        C_MATRIX.set(0, 0, -m2 * l1 * r2 * s2 * theta2Dot);
        C_MATRIX.set(0, 1, -m2 * l1 * r2 * s2 * (theta1Dot + theta2Dot));
        C_MATRIX.set(1, 0, m2 * l1 * r2 * s2 * theta1Dot);
        C_MATRIX.set(1, 1, 0);
    }

    /**
     * Calculates the Tg vector.
     *
     * @param theta1 the angle of the shoulder joint. [rad]
     * @param theta2 the angle of the elbow joint. [rad]
     */
    public void updateTgVector(double theta1, double theta2) {
        double c1 = Math.cos(theta1);
        double c12 = Math.cos(theta1 + theta2);

        Tg_VECTOR.set(0, 0, (m1 * r1 + m2 * l1) * g1 * c1 + m2 * r2 * g2 * c12);
        Tg_VECTOR.set(1, 0, m2 * r2 * g2 * c12);
    }

    /**
     * Calculates the voltage feedforward required to move the arm in a certain state.
     *
     * @param theta1 the angle of the shoulder joint. [rad]
     * @param theta2 the angle of the elbow joint. [rad]
     * @param theta1Dot the angular velocity of the shoulder joint. [rad/s]
     * @param theta2Dot the angular velocity of the elbow joint. [rad/s]
     * @param theta1DotDot the angular acceleration of the shoulder joint. [rad/s^2]
     * @param theta2DotDot the angular acceleration of the elbow joint. [rad/s^2]
     * @return the voltage feedforward required to move the arm in the given state. [V]
     */
    public ArmFeedForward calculateFeedForward(double theta1, double theta2, double theta1Dot, double theta2Dot, double theta1DotDot, double theta2DotDot) {
        updateMMatrix(theta2);
        updateCMatrix(theta2, theta1Dot, theta2Dot);
        updateTgVector(theta1, theta2);

        var thetaDotVector = new SimpleMatrix(2, 1, MatrixType.DDRM);
        thetaDotVector.set(0, 0, theta1Dot);
        thetaDotVector.set(1, 0, theta2Dot);

        var thetaDotDotVector = new SimpleMatrix(2, 1, MatrixType.DDRM);
        thetaDotDotVector.set(0, 0, theta1DotDot);
        thetaDotDotVector.set(1, 0, theta2DotDot);

        var M = M_MATRIX.mult(thetaDotDotVector);
        var C = C_MATRIX.mult(thetaDotVector);
        var Kb = Kb_MATRIX.mult(thetaDotVector);
        var B_INV = B_MATRIX.invert();

        var u = B_INV.mult(M.plus(C).plus(Kb).plus(Tg_VECTOR));

        return new ArmFeedForward(u.get(0, 0), u.get(1, 0));
    }

    /**
     * Calculates the voltage feedforward required to move the arm in a certain state.
     */
    public static class ArmFeedForward {
        double shoulderFeedForward;
        double elbowFeedForward;

        /**
         * Constructor.
         *
         * @param shoulderFeedForward the voltage feedforward required to move the shoulder joint. [V]
         * @param elbowFeedForward the voltage feedforward required to move the elbow joint. [V]
         */
        public ArmFeedForward(double shoulderFeedForward, double elbowFeedForward) {
            this.shoulderFeedForward = shoulderFeedForward;
            this.elbowFeedForward = elbowFeedForward;
        }
    }
}
