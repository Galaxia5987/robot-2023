package frc.robot.subsystems.arm;

public class ArmConstants { // TODO: Update all constants at zero

    // Arm constants
    public static final double SHOULDER_GEARING = 0; // Arbitrary units
    public static final double SHOULDER_MASS = 0; // [kg]
    public static final double SHOULDER_LENGTH = 0; // [m]
    public static final double SHOULDER_MOMENT_OF_INERTIA = 0; // [kg*m^2]
    public static final double SHOULDER_CM_RADIUS = 0; // [m]
    public static final int SHOULDER_NUMBER_OF_MOTORS = 0; // Arbitrary units

    public static final double ELBOW_GEARING = 0; // Arbitrary units
    public static final double ELBOW_MASS = 0; // [kg]
    public static final double ELBOW_LENGTH = 0; // [m]
    public static final double ELBOW_MOMENT_OF_INERTIA = 0; // [kg*m^2]
    public static final double ELBOW_CM_RADIUS = 0; // [m]
    public static final int ELBOW_NUMBER_OF_MOTORS = 0; // Arbitrary units

    public static final InertialConstants.JointConstants SHOULDER_JOINT_CONSTANTS = new InertialConstants.JointConstants(
            SHOULDER_MASS, SHOULDER_LENGTH, SHOULDER_MOMENT_OF_INERTIA, SHOULDER_CM_RADIUS, SHOULDER_GEARING, SHOULDER_NUMBER_OF_MOTORS);
    public static final InertialConstants.JointConstants ELBOW_JOINT_CONSTANTS = new InertialConstants.JointConstants(
            ELBOW_MASS, ELBOW_LENGTH, ELBOW_MOMENT_OF_INERTIA, ELBOW_CM_RADIUS, ELBOW_GEARING, ELBOW_NUMBER_OF_MOTORS);

    public static final InertialConstants ARM_CONSTANTS = new InertialConstants(SHOULDER_JOINT_CONSTANTS, ELBOW_JOINT_CONSTANTS);
}
