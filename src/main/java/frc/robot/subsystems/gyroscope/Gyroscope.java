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
        loggerInputs.zeroAngle = new Rotation2d();
    }

    @Override
    public void updateInputs() {
        loggerInputs.rawAngle = navx.getRotation2d();
        loggerInputs.angle = getAngle();
    }

    @Override
    public String getSubsystemName() {
        return "Gyroscope";
    }

    /**
     * Resets the angle of the navx to the current angle.
     */
    public void resetAngle() {
        resetAngle(new Rotation2d());
    }

    /**
     * Resets the angle of the navx to the current angle.
     *
     * @param angle the angle in -180 to 180 degrees coordinate system.
     */
    public void resetAngle(Rotation2d angle) {
        loggerInputs.zeroAngle = getRawAngle().minus(angle);
    }

    /**
     * Gets the current angle of the robot in respect to the start angle.
     *
     * @return the current angle of the robot in respect to the start angle.
     */
    public Rotation2d getAngle() {
        return getRawAngle().minus(loggerInputs.zeroAngle);
    }

    /**
     * Gets the raw angle from the navx.
     *
     * @return the angle of the robot in respect to the angle of the robot initiation time.
     */
    public Rotation2d getRawAngle() {
        return loggerInputs.rawAngle;
    }

    public AngleUtil.Angle getAngleObject() {
        return new AngleUtil.Angle(AngleUtil.UP_COUNTER_CLOCKWISE, getAngle().getDegrees());
    }
}
