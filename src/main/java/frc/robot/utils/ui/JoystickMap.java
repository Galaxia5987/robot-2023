package frc.robot.utils.ui;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;

public class JoystickMap implements ButtonMap {
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    public JoystickMap(Joystick leftJoystick, Joystick rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public void defaultDriveSignal(DriveSignal signal, SlewRateLimiter forwardRateLimiter, SlewRateLimiter strafeRateLimiter, SlewRateLimiter rotationRateLimiter) {
        double vx = -leftJoystick.getY();
        double vy = -leftJoystick.getX();
        double omega = -rightJoystick.getX();

        vx = forwardRateLimiter.calculate(vx);
        vy = strafeRateLimiter.calculate(vy);
        omega = rotationRateLimiter.calculate(omega);

        double magnitude = Math.hypot(vx, vy);
        double angle = Math.atan2(vy, vx);
        magnitude = MathUtil.applyDeadband(magnitude, 0.1);
        vx = Math.cos(angle) * magnitude;
        vy = Math.sin(angle) * magnitude;
        omega = MathUtil.applyDeadband(omega, 0.1);

        signal.vx = vx * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        signal.vy = vy * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        signal.omega = omega * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        signal.centerOfRotation = new Translation2d();
        signal.fieldOriented = !leftJoystick.getTop();
    }

    @Override
    public boolean getDoubleSubstation() {
        return false;
    }

    @Override
    public boolean getSingleSubstation() {
        return false;
    }

    @Override
    public boolean getScoreGamePiece() {
        return false;
    }

    @Override
    public boolean getIntakeCone() {
        return false;
    }

    @Override
    public boolean getIntakeCube() {
        return false;
    }

    @Override
    public boolean lock() {
        return false;
    }

    @Override
    public boolean getChargeStation() {
        return false;
    }
}
