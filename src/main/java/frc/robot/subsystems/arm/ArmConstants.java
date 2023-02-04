package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmConstants { //TODO: find all constant values

    // motor configuration
    public static final double VOLT_COMP_SATURATION = 10; //[V]
    public static final boolean ENABLE_VOLT_COMPENSATION = true;
    public static final double MOTION_ACCELERATION = 0; // [rad/sec^2]
    public static final double MOTION_CRUISE_VELOCITY = 0; // [rad/sec]
    public static final double DEADBAND = 0.05; // [%]
    public static final double SETPOINT_DEADBAND = 0;
    public static final double TICKS_PER_RADIAN = 1024 / (Math.PI * 2);
    public static final TalonFXInvertType MAIN_CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType AUX_CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = 0;
    public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = 0;

    //PID
    public static final double shoulderP = 0.1;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 0.0;
    public static final double elbowP = 0.1;
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;

    //arm positions
    public static final Translation2d ABOVE_GAME_PIECE = new Translation2d();
    public static final Translation2d FEEDER_POSITION = new Translation2d();
    public static final Translation2d RETRACTED_POSITION = new Translation2d();
    public static final Translation2d UPPER_CONE_SCORING = new Translation2d();
    public static final Translation2d MIDDLE_CONE_SCORING = new Translation2d();
    public static final Translation2d UPPER_CUBE_SCORING = new Translation2d();
    public static final Translation2d MIDDLE_CUBE_SCORING = new Translation2d();
    public static final Translation2d FLOOR_SCORING = new Translation2d();

    //shoulder physics
    public static final double SHOULDER_GEARING = 0; // Arbitrary units
    public static final double SHOULDER_MASS = 0; // [kg]
    public static final double SHOULDER_LENGTH = 0; // [m]
    public static final double SHOULDER_MOMENT_OF_INERTIA = 0; // [kg*m^2]
    public static final double SHOULDER_CENTER_OF_MASS_RADIUS = 0; // [m]
    public static final int SHOULDER_NUMBER_OF_MOTORS = 2; // Arbitrary units
    public static final double SHOULDER_ARM_LENGTH = 0; //[m]

    //elbow physics
    public static final double ELBOW_GEARING = 0; // Arbitrary units
    public static final double ELBOW_MASS = 0; // [kg]
    public static final double ELBOW_LENGTH = 0; // [m]
    public static final double ELBOW_MOMENT_OF_INERTIA = 0; // [kg*m^2]
    public static final double ELBOW_CENTER_OF_MASS_RADIUS = 0; // [m]
    public static final int ELBOW_NUMBER_OF_MOTORS = 2; // Arbitrary units
    public static final double ELBOW_ARM_LENGTH = 0; //[m]

    public static final Translation2d ARM_DEFAULT_POSITION = new Translation2d(0, 0);

    public static final SystemConstants.JointConstants SHOULDER_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            SHOULDER_MASS, SHOULDER_LENGTH, SHOULDER_MOMENT_OF_INERTIA, SHOULDER_CENTER_OF_MASS_RADIUS, SHOULDER_GEARING, SHOULDER_NUMBER_OF_MOTORS);
    public static final SystemConstants.JointConstants ELBOW_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            ELBOW_MASS, ELBOW_LENGTH, ELBOW_MOMENT_OF_INERTIA, ELBOW_CENTER_OF_MASS_RADIUS, ELBOW_GEARING, ELBOW_NUMBER_OF_MOTORS);

    public static final SystemConstants ARM_CONSTANTS = new SystemConstants(SHOULDER_JOINT_CONSTANTS, ELBOW_JOINT_CONSTANTS);
}