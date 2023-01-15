package frc.robot.utils.ui;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.drivetrain.DriveSignal;

public class JoystickMap implements ButtonMap {
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    public JoystickMap(Joystick leftJoystick, Joystick rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public DriveSignal defaultDriveSignal(SlewRateLimiter forwardRateLimiter, SlewRateLimiter strafeRateLimiter, SlewRateLimiter rotationRateLimiter) {
        double vx = leftJoystick.getY();
        double vy = leftJoystick.getX();
        double omega = rightJoystick.getX();

        vx = forwardRateLimiter.calculate(vx);
        vy = strafeRateLimiter.calculate(vy);
        omega = rotationRateLimiter.calculate(omega);

        return new DriveSignal(vx, vy, omega,
                new Translation2d(),
                leftJoystick.getTrigger());
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
    public boolean getChargeStation() {
        return false;
    }
}
