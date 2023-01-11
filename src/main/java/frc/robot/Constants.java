package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.autonomous.DrivetrainFeedforwardConstants;

public final class Constants {
    public static final int TALON_TIMEOUT = 10;
    public static final int NOMINAL_VOLTAGE = 12;
    public static final Pose2d HUB_POSE = new Pose2d(8.23, 4.115, new Rotation2d());

    // Swerve

    public static class SwerveDrive {
        public static final double TICKS_PER_ROTATION = 2048;
        public static final int[] OFFSETS = {20158, 6551, 16034, 1662};

        public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.595;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.595;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 7;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 4;
        public static final int REAR_LEFT_MODULE_DRIVE_MOTOR_ID = 1;
        public static final int REAR_LEFT_MODULE_STEER_MOTOR_ID = 6;
        public static final int REAR_RIGHT_MODULE_DRIVE_MOTOR_ID = 5;
        public static final int REAR_RIGHT_MODULE_STEER_MOTOR_ID = 2;
        public static final double DRIVE_REDUCTION = (1 / 2.0) * (22.0 / 24.0) * (15.0 / 45.0);
        public static final double ANGLE_GEAR_RATIO = (14.0 / 72.0) * 0.5;
        public static final double WHEEL_DIAMETER = 0.11140846016;
        // kP, kI, kD, kF, sCurveStrength, cruiseVelocity, acceleration, allowableError,
        // maxIntegralAccum, peakOutput
        public static final double[] FRONT_LEFT_MOTION_MAGIC_CONFIGS = {0.8, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
        public static final double[] FRONT_RIGHT_MOTION_MAGIC_CONFIGS = {0.8, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
        public static final double[] REAR_LEFT_MOTION_MAGIC_CONFIGS = {0.8, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
        public static final double[] REAR_RIGHT_MOTION_MAGIC_CONFIGS = {0.8, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                DRIVE_REDUCTION *
                WHEEL_DIAMETER * Math.PI + 0.2;
        public static final double MAX_LINEAR_ACCELERATION = MAX_VELOCITY_METERS_PER_SECOND / 2;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.0;

        public static final double XY_SLEW_RATE_LIMIT = 3.0;
        public static final double ROTATION_SLEW_RATE_LIMIT = 6.0;
        public static boolean FRONT_LEFT_DRIVE_INVERTED = true;
        public static boolean FRONT_LEFT_ANGLE_INVERTED = true;
        public static boolean FRONT_LEFT_ANGLE_SENSOR_PHASE = false;
        public static boolean FRONT_RIGHT_DRIVE_INVERTED = true;
        public static boolean FRONT_RIGHT_ANGLE_INVERTED = true;
        public static boolean FRONT_RIGHT_ANGLE_SENSOR_PHASE = false;
        public static boolean REAR_LEFT_DRIVE_INVERTED = true;
        public static boolean REAR_LEFT_ANGLE_INVERTED = true;
        public static boolean REAR_LEFT_ANGLE_SENSOR_PHASE = false;
        public static boolean REAR_RIGHT_DRIVE_INVERTED = true;
        public static boolean REAR_RIGHT_ANGLE_INVERTED = true;
        public static boolean REAR_RIGHT_ANGLE_SENSOR_PHASE = false;

        public static double SMOOTHING_FACTOR = 2;

        public static double TORNADO_SPIN_DISTANCE = 0.4;

        public static double AUTO_XY_Kp = 50;
        public static double AUTO_XY_Ki = 0.0;
        public static double AUTO_XY_Kd = 0.0;
        public static double AUTO_ROTATION_Kp = 0;
        public static double AUTO_ROTATION_Ki = 0.0;
        public static double AUTO_ROTATION_Kd = 0.0;
        public static double AUTO_XY_VELOCITY_FEEDFORWARD = 0;
        public static double AUTO_XY_ACCELERATION_FEEDFORWARD = 0;
        public static double AUTO_XY_STATIC_FEEDFORWARD = 0;

        public static DrivetrainFeedforwardConstants translationConstants = new DrivetrainFeedforwardConstants(
                AUTO_XY_VELOCITY_FEEDFORWARD, AUTO_XY_ACCELERATION_FEEDFORWARD, AUTO_XY_STATIC_FEEDFORWARD
        );
        public static PIDConstants translationPidConstants = new PIDConstants(
                AUTO_XY_Kp, AUTO_XY_Ki, AUTO_XY_Kd
        );
        public static PIDConstants rotationPidConstants = new PIDConstants(
                AUTO_ROTATION_Kp, AUTO_ROTATION_Ki, AUTO_ROTATION_Kd
        );

        public static double CHARGING_STATION_BALANCE_Kp = 0.5;
        public static double CHARGING_STATION_BALANCE_Ki = 0;
        public static double CHARGING_STATION_BALANCE_Kd = 0;
        public static double CHARGING_STATION_BALANCE_Kf = 0;
        public static double CHARGING_STATION_BALANCE_VELOCITY = 0.5;
        public static double CHARGING_STATION_BALANCE_ACCELERATION = 0.25;
        public static TrapezoidProfile.Constraints CHARGING_STATION_BALANCE_CONSTRAINTS = new TrapezoidProfile.Constraints(
                CHARGING_STATION_BALANCE_VELOCITY, CHARGING_STATION_BALANCE_ACCELERATION);
    }

}
