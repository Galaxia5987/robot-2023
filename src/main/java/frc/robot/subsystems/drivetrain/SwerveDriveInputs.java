package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveDriveInputs {
    public double supplyCurrent;
    public double statorCurrent;

    // x, y, omega
    public double[] currentSpeeds = {0, 0, 0};
    public double[] desiredSpeeds = {0, 0, 0};

    public double[] absolutePositions = new double[4];

    public double rawYaw;
    public double yaw;
    public double gyroOffset;

    public double angleFF;
    public double error;
    public double pidSetpoint;

    public double[] botPose = {0, 0, 0}; //x, y, rotation
}
