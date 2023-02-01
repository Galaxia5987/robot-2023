package subsystems.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SPI;
import subsystems.LoggedSubsystem;
import frc.robot.utils.math.AngleUtil;

public class Gyroscope extends LoggedSubsystem<GyroscopeLogInputs> {
    private final AHRS navx;
    private double zeroPitch;
    private double zeroRoll;
    private boolean zeroInitialized = false;

    public Gyroscope() {
        super(new GyroscopeLogInputs());
        navx = new AHRS(SPI.Port.kMXP);
        loggerInputs.zeroYaw = new Rotation2d();
    }

    @Override
    public void periodic() {
        if (!zeroInitialized && navx.isConnected()) {
            zeroRoll = navx.getRoll();
            zeroPitch = navx.getPitch();
            zeroInitialized = true;
        }
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
        loggerInputs.zeroYaw = new Rotation2d();
        navx.reset();
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
     * Gets the object representing all three angles of the gyro.
     *
     * @return object representing all three of the angles of the gyro.
     */
    public Rotation3d getAll() {
        return new Rotation3d(loggerInputs.roll.getRadians(),
                loggerInputs.pitch.getRadians(),
                loggerInputs.yaw.getRadians());
    }
}
