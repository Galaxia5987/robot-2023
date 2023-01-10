package frc.robot.subsystems.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.math.AngleUtil;

public class Gyroscope extends LoggedSubsystem<GyroscopeLogInputs> {
    private final AHRS navx;

    public Gyroscope() {
        super(new GyroscopeLogInputs());
        navx = new AHRS(SPI.Port.kMXP);
        loggerInputs.zeroYaw = new Rotation2d();
    }

    @Override
    public void updateInputs() {
        loggerInputs.rawYaw = navx.getRotation2d();
        loggerInputs.yaw = getYaw();
        loggerInputs.pitch = Rotation2d.fromDegrees(navx.getPitch());
        loggerInputs.roll = Rotation2d.fromDegrees(navx.getRoll());
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

    public Rotation2d getPitch() {
        return loggerInputs.pitch;
    }

    public Rotation2d getRoll() {
        return loggerInputs.roll;
    }

    public YawPitchRoll getYawPitchRoll() {
        return new YawPitchRoll(loggerInputs.yaw.getRadians(),
                loggerInputs.pitch.getRadians(),
                loggerInputs.roll.getRadians());
    }

    public record YawPitchRoll(double yaw, double pitch, double roll) {}
}
