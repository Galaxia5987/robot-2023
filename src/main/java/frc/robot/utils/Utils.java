package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Utils {
    public static final double EPSILON = 1e-9;

    /**
     * sets the value of the joystick to 0 if the value is less than the threshold
     *
     * @param val       the joystick value
     * @param threshold the threshold value
     * @return 0 if val is less than the threshold else val
     */
    public static double deadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return (val - Math.signum(val) * threshold) / (1 - threshold);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(double a, double b, double maxError) {
        return deadband(a - b, maxError) == 0;
    }

    public static double[] pose2dToArray(Pose2d pose) {
        return new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    }

    public static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
        return new double[]{speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond};
    }

    public static ChassisSpeeds arrayToChassisSpeeds(double[] array) {
        return new ChassisSpeeds(array[0], array[1], array[2]);
    }

    /**
     * Converts the relative angles to pitch relative to the coordinate system of the field,
     * aka the absolute pitch. This value can be used to balance on the charge station.
     * <p>
     * The calculations were made by multiplying the rotation matrices of yaw pitch and roll
     * in that order by the x unit vector (1, 0, 0).
     * See {@link <a href="https://www.tu-chemnitz.de/informatik/service/ib/pdf/CSR-21-01.pdf">...</a>} at page 11,
     * where the coordinate system of a UAV is described.
     * <p>
     * The robot coordinate system is in the opposite direction in all axes. Hence, the angles were multiplied by -1.
     *
     * @param yaw   the yaw angle from the gyro. [rad]
     * @param pitch the pitch angle from the gyro. [rad]
     * @param roll  the roll angle from the gyro. [rad]
     * @return the absolute pitch angle. [rad]
     */
    public static double relativeAnglesToAbsolutePitch(double yaw, double pitch, double roll) {
        return Math.atan(
                (Math.sin(roll) * Math.sin(yaw) + Math.cos(roll) * Math.sin(pitch) * Math.cos(yaw)) / (Math.cos(pitch) * Math.cos(yaw))
        );
    }

    public static boolean speedsEpsilonEquals(ChassisSpeeds speeds) {
        return epsilonEquals(speeds.vxMetersPerSecond, 0) &&
                epsilonEquals(speeds.vyMetersPerSecond, 0) &&
                epsilonEquals(speeds.omegaRadiansPerSecond, 0);
    }
}