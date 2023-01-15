package frc.robot.utils.ui;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.drivetrain.DriveSignal;

public interface ButtonMap {

    DriveSignal defaultDriveSignal(SlewRateLimiter forwardRateLimiter, SlewRateLimiter strafeRateLimiter, SlewRateLimiter rotationRateLimiter);

    boolean getDoubleSubstation();

    boolean getSingleSubstation();

    boolean getScoreGamePiece();

    boolean getIntakeCone();

    boolean getIntakeCube();

    boolean getChargeStation();
}
