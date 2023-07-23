package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;

public class GyroIOReal implements GyroIO {
    private final AHRS gyro;
    private double gyroOffset = 0;

    public GyroIOReal() {
        this.gyro = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public double getYaw() {
        return getRawYaw() - gyroOffset;
    }

    @Override
    public double getRawYaw() {
        return -MathUtil.angleModulus(Math.toRadians(gyro.getAngle()));
    }

    @Override
    public void resetGyro(double angle) {
        gyroOffset = angle + getRawYaw();
    }

    @Override
    public void updateInputs(SwerveDriveInputs inputs) {
        inputs.gyroOffset = gyroOffset;
    }
}
