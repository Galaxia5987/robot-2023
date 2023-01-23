package frc.robot.subsystems.arm;

public class ArmConstants {

    // Arm constants
    public static final double CONFIG_VOLT_COMP = 10; //[voltage]
    public static final boolean ENABLE_VOLT_COMPENSATION = true;
    public static final boolean clockWise = true;

    public static final double shoulderP = 0.1;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 0.0;
    public static final double elbowP = 0.1;
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;

    public static final double TICKS_PER_RADIAN = 1024 / (Math.PI * 2);

    public static final double SHOULDER_GEARING = 0; // Arbitrary units
    public static final double SHOULDER_MASS = 0; // [kg]
    public static final double SHOULDER_LENGTH = 0; // [m]
    public static final double SHOULDER_MOMENT_OF_INERTIA = 0; // [kg*m^2]
    public static final double SHOULDER_CM_RADIUS = 0; // [m]
    public static final int SHOULDER_NUMBER_OF_MOTORS = 2; // Arbitrary units
    public static final double SHOULDER_ARM_LENGTH = 0; //[m]

    public static final double ELBOW_GEARING = 0; // Arbitrary units
    public static final double ELBOW_MASS = 0; // [kg]
    public static final double ELBOW_LENGTH = 0; // [m]
    public static final double ELBOW_MOMENT_OF_INERTIA = 0; // [kg*m^2]
    public static final double ELBOW_CM_RADIUS = 0; // [m]
    public static final int ELBOW_NUMBER_OF_MOTORS = 2; // Arbitrary units
    public static final double ELBOW_ARM_LENGTH = 0; //[m]

    public static final SystemConstants.JointConstants SHOULDER_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            SHOULDER_MASS, SHOULDER_LENGTH, SHOULDER_MOMENT_OF_INERTIA, SHOULDER_CM_RADIUS, SHOULDER_GEARING, SHOULDER_NUMBER_OF_MOTORS);
    public static final SystemConstants.JointConstants ELBOW_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            ELBOW_MASS, ELBOW_LENGTH, ELBOW_MOMENT_OF_INERTIA, ELBOW_CM_RADIUS, ELBOW_GEARING, ELBOW_NUMBER_OF_MOTORS);

    public static final SystemConstants ARM_CONSTANTS = new SystemConstants(SHOULDER_JOINT_CONSTANTS, ELBOW_JOINT_CONSTANTS);
}