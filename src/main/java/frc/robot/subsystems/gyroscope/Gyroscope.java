package frc.robot.subsystems.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.math.AngleUtil;

public class Gyroscope extends LoggedSubsystem<GyroscopeLogInputs> {
    private static Gyroscope INSTANCE;
    private final AHRS navx;
    private final boolean zeroInitialized = false;
    private Rotation2d zeroPitch = new Rotation2d();
    private double zeroRoll;

    private Gyroscope() {
        super(new GyroscopeLogInputs());
        navx = new AHRS(SPI.Port.kMXP);
        loggerInputs.zeroYaw = new Rotation2d();
        navx.reset();
    }

    public static Gyroscope getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Gyroscope();
        }
        return INSTANCE;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void updateInputs() {
        loggerInputs.rawYaw = navx.getRotation2d();
        loggerInputs.yaw = getYaw();
        loggerInputs.pitch = zeroPitch.minus(Rotation2d.fromDegrees(navx.getPitch()));
        loggerInputs.roll = new Rotation2d(navx.getRoll() - zeroRoll);
    }

    @Override
    public String getSubsystemName() {
        return "Gyroscope";
    }

    /**
     * Resets the yaw of the navx to the current yaw.
     */
    public void resetYaw() {
        loggerInputs.zeroYaw = getRawYaw();
    }

    /**
     * Resets the yaw of the navx to the current yaw.
     *
     * @param yaw the yaw in -180 to 180 degrees coordinate system.
     */
    public void resetYaw(Rotation2d yaw) {
        loggerInputs.zeroYaw = getRawYaw().minus(yaw);
        zeroPitch = Rotation2d.fromDegrees(navx.getPitch());
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
