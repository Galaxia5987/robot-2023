package frc.robot.subsystems.drivetrain;

public interface GyroIO {
    void updateInputs(SwerveDriveInputs inputs);

    double getYaw();

    default double getRawYaw() {
        return 0;
    }

    void resetGyro(double angle);
}
