package frc.robot.subsystems.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.math.AngleUtil;

public class Gyroscope extends LoggedSubsystem<GyroscopeLogInputs> {
    private final AHRS navx;
    private double zeroPitch;
    private double zeroRoll;
    private boolean zeroInitialized = false;

    @Override
    public void periodic() {
        if (!zeroInitialized && navx.isConnected()) {
            zeroRoll = navx.getRoll();
            zeroPitch = navx.getPitch();
            zeroInitialized = true;
        }
    }

    public Gyroscope() {
        super(new GyroscopeLogInputs());
        navx = new AHRS(SPI.Port.kMXP);
        loggerInputs.zeroYaw = new Rotation2d();
    }

    @Override
    public void updateInputs() {
        loggerInputs.rawYaw = navx.getRotation2d();
        loggerInputs.yaw = getYaw();
        loggerInputs.pitch = Rotation2d.fromDegrees(Math.toDegrees(zeroPitch - navx.getPitch()));
        loggerInputs.roll = Rotation2d.fromDegrees(Math.toDegrees(navx.getRoll() - zeroRoll));
    }

    @Override
    public String getSubsystemName() {
        return "Gyroscope";
    }

    /**
     * Resets the yaw of the navx to the current yaw.
     */
    public void resetYaw() {
        resetYaw(new Rotation2d());
    }

    /**
     * Resets the yaw of the navx to the current yaw.
     *
     * @param yaw the yaw in -180 to 180 degrees coordinate system.
     */
    public void resetYaw(Rotation2d yaw) {
        loggerInputs.zeroYaw = getRawYaw().minus(yaw);
    }

    /**
     * Gets the current yaw of the robot in respect to the start yaw.
     *
     * @return the current yaw of the robot in respect to the start yaw.
     */
    public Rotation2d getYaw() {
        return getRawYaw().minus(loggerInputs.zeroYaw);
    }

    /**
     * Gets the raw yaw from the navx.
     *
     * @return the yaw of the robot in respect to the yaw of the robot initiation time.
     */
    public Rotation2d getRawYaw() {
        return loggerInputs.rawYaw;
    }

    public AngleUtil.Angle getYawObject() {
        return new AngleUtil.Angle(AngleUtil.UP_COUNTER_CLOCKWISE, getYaw().getDegrees());
    }

    /**
     * Gets the pitch of the robot.
     *
     * @return the pitch of the robot. [rad]
     */
    public Rotation2d getPitch() {
        return loggerInputs.pitch;
    }

    /**
     * Gets the roll of the robot.
     *
     * @return the roll of the robot. [rad]
     */
    public Rotation2d getRoll() {
        return loggerInputs.roll;
    }

    /**
     * Gets the yaw pitch roll object representing the angles of the gyro.
     *
     * @return the yaw pitch roll object representing the angles of the gyro.
     *       The angles are in radians. See YawPitchRoll record documentation below for more information.
     */
    public YawPitchRoll getYawPitchRoll() {
        return new YawPitchRoll(loggerInputs.yaw.getRadians(),
                loggerInputs.pitch.getRadians(),
                loggerInputs.roll.getRadians());
    }

    /**
     * This record represents the angle of the gyro in all three axes.
     *
     * Params:
     * - yaw: the angle of the robot in the x-y plane. [rad]
     * - pitch: the angle of the robot in the x-z plane. [rad]
     * - roll: the angle of the robot in the y-z plane. [rad]
     */
    public record YawPitchRoll(double yaw, double pitch, double roll) {}
}
