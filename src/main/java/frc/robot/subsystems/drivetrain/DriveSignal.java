package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This is the main drive signal record.
 *
 * @param vx is the forward velocity. [m/s]
 * @param vy is the strafe velocity. [m/s]
 * @param omega is the rotation velocity. [rad/s]
 * @param centerOfRotation is the center of rotation to rotate around. This is mostly (0, 0),
 *                         except when doing a tornado spin. ([m], [m])
 * @param fieldOriented is whether the swerve should drive field oriented.
 */
public record DriveSignal(double vx, double vy, double omega, Translation2d centerOfRotation, boolean fieldOriented) {

    public DriveSignal(ChassisSpeeds speeds, Translation2d centerOfRotation, boolean fieldOriented) {
        this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, centerOfRotation, fieldOriented);
    }

    public ChassisSpeeds speeds() {
        return new ChassisSpeeds(vx, vy, omega);
    }
}
