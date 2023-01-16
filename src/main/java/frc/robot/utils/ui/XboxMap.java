package frc.robot.utils.ui;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.utils.Utils;

public class XboxMap implements ButtonMap {
    private final XboxController xboxController;

    public XboxMap(XboxController xboxController) {
        this.xboxController = xboxController;
    }

    @Override
    public DriveSignal defaultDriveSignal(SlewRateLimiter forwardRateLimiter, SlewRateLimiter strafeRateLimiter, SlewRateLimiter rotationRateLimiter) {
        double vx = -xboxController.getLeftY();
        double vy = -xboxController.getLeftX();
        double omega = -xboxController.getRightX();

        vx = forwardRateLimiter.calculate(vx);
        vy = strafeRateLimiter.calculate(vy);
        omega = rotationRateLimiter.calculate(omega);

        double magnitude = Math.hypot(vx, vy);
        double angle = Math.atan2(vy, vx);
        magnitude = Utils.deadband(magnitude, 0.1);
        vx = Math.cos(angle) * magnitude;
        vy = Math.sin(angle) * magnitude;
        omega = Utils.deadband(omega, 0.1);

        return new DriveSignal(vx * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                vy * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                omega * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                new Translation2d(),
                xboxController.getLeftTriggerAxis() <= 0.1);
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
