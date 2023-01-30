package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveSignal {
    public double vx;
    public double vy;
    public double omega;
    public Translation2d centerOfRotation;
    public boolean fieldOriented;

    /**
     * This is the main drive signal constructor.
     *
     * @param vx               is the forward velocity. [m/s]
     * @param vy               is the strafe velocity. [m/s]
     * @param omega            is the rotation velocity. [rad/s]
     * @param centerOfRotation is the center of rotation to rotate around. This is mostly (0, 0),
     *                         except when doing a tornado spin. ([m], [m])
     * @param fieldOriented    is whether the swerve should drive field oriented.
     */
    public DriveSignal(double vx, double vy, double omega, Translation2d centerOfRotation, boolean fieldOriented) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        this.centerOfRotation = centerOfRotation;
        this.fieldOriented = fieldOriented;
    }

    public DriveSignal(ChassisSpeeds speeds, Translation2d centerOfRotation, boolean fieldOriented) {
        this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, centerOfRotation, fieldOriented);
    }

    public ChassisSpeeds speeds() {
        return new ChassisSpeeds(vx, vy, omega);
    }
}
