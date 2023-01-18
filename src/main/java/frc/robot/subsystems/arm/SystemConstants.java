package frc.robot.subsystems.arm;

public class SystemConstants {
    private final JointConstants shoulderJointConstants;
    private final JointConstants elbowJointConstants;

    public SystemConstants(JointConstants shoulderJointConstants, JointConstants elbowJointConstants) {
        this.shoulderJointConstants = shoulderJointConstants;
        this.elbowJointConstants = elbowJointConstants;
    }

    public SystemConstants(double shoulderMass, double shoulderLength, double shoulderMomentOfInertia, double shoulderCmRadius, double shoulderGearing, int shoulderNumMotors,
                           double elbowMass, double elbowLength, double elbowMomentOfInertia, double elbowCmRadius, double elbowGearing, int elbowNumMotors) {
        this(new JointConstants(shoulderMass, shoulderLength, shoulderMomentOfInertia, shoulderCmRadius, shoulderGearing, shoulderNumMotors),
             new JointConstants(elbowMass, elbowLength, elbowMomentOfInertia, elbowCmRadius, elbowGearing, elbowNumMotors));
    }

    public JointConstants getShoulderJointConstants() {
        return shoulderJointConstants;
    }

    public JointConstants getElbowJointConstants() {
        return elbowJointConstants;
    }

    public static class JointConstants {
        public double mass;
        public double length;
        public double momentOfInertia;
        public double cmRadius;
        public double gearing;
        public int numberOfMotors;

        public JointConstants(double mass, double length, double momentOfInertia, double cmRadius, double gearing, int numberOfMotors) {
            this.mass = mass;
            this.length = length;
            this.momentOfInertia = momentOfInertia;
            this.cmRadius = cmRadius;
            this.gearing = gearing;
            this.numberOfMotors = numberOfMotors;
        }
    }
}
