package frc.robot.subsystems.drivetrain;

import frc.robot.utils.math.differential.Integral;

public class GyroIOSim implements GyroIO {
    private Integral yaw = new Integral(0, 0);

    @Override
    public double getYaw() {
        return yaw.get();
    }

    @Override
    public void resetGyro(double angle) {
        yaw.override(angle);
    }

    @Override
    public void updateInputs(SwerveDriveInputs inputs) {
        yaw.update(inputs.currentSpeeds[2]);
    }
}
