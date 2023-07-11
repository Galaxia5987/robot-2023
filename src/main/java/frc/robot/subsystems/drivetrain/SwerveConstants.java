package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.robot.autonomous.DrivetrainFeedforwardConstants;

public class SwerveConstants {
    public static final double TICKS_PER_ROTATION = 2048;
    public static final int[] OFFSETS = {205, 1896, 6992, 9758};

    public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.51594;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.66594;

    public static final double DRIVE_REDUCTION = (1 / 2.0) * (22.0 / 24.0) * (15.0 / 45.0);
    public static final double ANGLE_GEAR_RATIO = (14.0 / 72.0) * 0.5;
    public static final double WHEEL_DIAMETER = 0.1023679821;

    public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT_CONFIG = new StatorCurrentLimitConfiguration(
            true, 50, 2, 0.02);
    public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(
            true, 50, 2, 0.02);
    public static final double NEUTRAL_DEADBAND = 0.1;

    // kP, kI, kD, kF, sCurveStrength, cruiseVelocity, acceleration, allowableError,
    // maxIntegralAccum, peakOutput
    public static final double[] FRONT_LEFT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
    public static final double[] FRONT_RIGHT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
    public static final double[] REAR_LEFT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
    public static final double[] REAR_RIGHT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            DRIVE_REDUCTION *
            WHEEL_DIAMETER * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.0;

    public static final double MAX_VELOCITY_AUTO = 4.0;
    public static final double MAX_ACCELERATION_AUTO = 2.5;

    public static final double XY_SLEW_RATE_LIMIT = 3.0;
    public static final double ROTATION_SLEW_RATE_LIMIT = 4.5;

    public static double AUTO_X_Kp = 3.20;
    public static double AUTO_X_Ki = 0.0;
    public static double AUTO_X_Kd = 0.73;
    public static double AUTO_X_Kf = 0.35;
    public static double AUTO_Y_Kp = 3.20;
    public static double AUTO_Y_Ki = 0.0;
    public static double AUTO_Y_Kd = 0.6;
    public static double AUTO_Y_Kf = 0.35;
    public static double AUTO_ROTATION_Kp = 10;
    public static double AUTO_ROTATION_Ki = 0.0;
    public static double AUTO_ROTATION_Kd = 0.0;
    public static double AUTO_ROTATION_Kf = 0.0;

    public static double TARGET_Y_Kp = 2.5;
    public static double TARGET_Y_Ki = 0.0;
    public static double TARGET_Y_Kd = 0.3;
    public static double TARGET_Y_Kf = 0.4;
    public static double TARGET_X_Kp = 2.5;
    public static double TARGET_X_Ki = 0.0;
    public static double TARGET_X_Kd = 0.3;
    public static double TARGET_X_Kf = 0.4;
    public static double TARGET_ROTATION_Kp = 6.0;
    public static double TARGET_ROTATION_Ki = 0.0;
    public static double TARGET_ROTATION_Kd = 0.0;
    public static double TARGET_ROTATION_Kf = 0.8;

    public static double TARGET_XY_Kp = 2.0;
    public static double TARGET_XY_Ki = 0.0;
    public static double TARGET_XY_Kd = 0.0;
    public static double TARGET_XY_Kf = 0.8;
    public static double AUTO_XY_VELOCITY_FEEDFORWARD = 0.33031;
    public static double AUTO_XY_ACCELERATION_FEEDFORWARD = 0.080553;
    public static double AUTO_XY_STATIC_FEEDFORWARD = 0.21189;
    public static double AUTO_ROTATION_VELOCITY_FEEDFORWARD = 0.3488;
    public static double AUTO_ROTATION_ACCELERATION_FEEDFORWARD = 0.02718;
    public static double AUTO_ROTATION_STATIC_FEEDFORWARD = 0.19735;

    public static DrivetrainFeedforwardConstants TRANSLATION_FF_CONSTANTS = new DrivetrainFeedforwardConstants(
            AUTO_XY_VELOCITY_FEEDFORWARD, AUTO_XY_ACCELERATION_FEEDFORWARD, AUTO_XY_STATIC_FEEDFORWARD
    );
    public static DrivetrainFeedforwardConstants ROTATION_FF_CONSTANTS = new DrivetrainFeedforwardConstants(
            AUTO_ROTATION_VELOCITY_FEEDFORWARD, AUTO_ROTATION_ACCELERATION_FEEDFORWARD, AUTO_ROTATION_STATIC_FEEDFORWARD
    );

    public static double CHARGING_STATION_BALANCE_Kp = 0;
    public static double CHARGING_STATION_BALANCE_Ki = 0;
    public static double CHARGING_STATION_BALANCE_Kd = 0;
    public static double CHARGING_STATION_BALANCE_Kf = 1;

    public static double FORWARD_BALANCE_TIME = 1.3;
    public static double BACKWARD_BALANCE_TIME = 1.1;
}
