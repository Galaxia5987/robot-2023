package frc.robot.utils.ui;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.drivetrain.DriveSignal;

public class XboxMap implements ButtonMap {
    private final XboxController xboxController;

    public XboxMap(XboxController xboxController) {
        this.xboxController = xboxController;
    }

    @Override
    public DriveSignal defaultDriveSignal(SlewRateLimiter forwardRateLimiter, SlewRateLimiter strafeRateLimiter, SlewRateLimiter rotationRateLimiter) {
        double vx = xboxController.getLeftY();
        double vy = xboxController.getLeftX();
        double omega = xboxController.getRightX();

        vx = forwardRateLimiter.calculate(vx);
        vy = strafeRateLimiter.calculate(vy);
        omega = rotationRateLimiter.calculate(omega);

        return new DriveSignal(vx, vy, omega,
                new Translation2d(),
                xboxController.getLeftTriggerAxis() > 0.1);
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
