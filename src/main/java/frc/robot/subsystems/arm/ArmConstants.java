package frc.robot.subsystems.arm;

public class ArmConstants { // TODO: Update all constants at zero, check if clockWise is inverted

    // Arm constants
    public static final double CONFIG_VOLT_COMP = 10; //[voltage]
    public static final boolean ENABLE_VOLT_COMPANSATION = true;
    public static final boolean clockWise = true;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double ENCODER_RATIO = 1/25;
    public static final double TICKS_PER_RADIAN = (4096/ENCODER_RATIO)/ (Math.PI*2);

    public static final double SHOULDER_GEARING = (66/16)*25; // Arbitrary units
    public static final double SHOULDER_MASS = 0.2859; // [kg]
    public static final double SHOULDER_LENGTH = 0.385; // [m]
    public static final double SHOULDER_MOMENT_OF_INERTIA = 0.0212405825*0.9; // [kg*m^2]
    public static final double SHOULDER_CM_RADIUS = 0.15869; // [m]
    public static final int SHOULDER_NUMBER_OF_MOTORS = 1; // Arbitrary units
    public static final double SHOULDER_ARM_LENGTH = 0.385; //[m]

    public static final double ELBOW_GEARING = (66/16)*25; // Arbitrary units
    public static final double ELBOW_MASS =0.2859; // [kg]
    public static final double ELBOW_LENGTH = 0.385; // [m]
    public static final double ELBOW_MOMENT_OF_INERTIA = 0.0212405825*0.9; // [kg*m^2]
    public static final double ELBOW_CM_RADIUS = 0.15869; // [m]
    public static final int ELBOW_NUMBER_OF_MOTORS = 1; // Arbitrary units
    public static final double ELBOW_ARM_LENGTH = 0.385; //[m]

    public static final SystemConstants.JointConstants SHOULDER_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            SHOULDER_MASS, SHOULDER_LENGTH, SHOULDER_MOMENT_OF_INERTIA, SHOULDER_CM_RADIUS, SHOULDER_GEARING, SHOULDER_NUMBER_OF_MOTORS);
    public static final SystemConstants.JointConstants ELBOW_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            ELBOW_MASS, ELBOW_LENGTH, ELBOW_MOMENT_OF_INERTIA, ELBOW_CM_RADIUS, ELBOW_GEARING, ELBOW_NUMBER_OF_MOTORS);

    public static final SystemConstants ARM_CONSTANTS = new SystemConstants(SHOULDER_JOINT_CONSTANTS, ELBOW_JOINT_CONSTANTS);
}
