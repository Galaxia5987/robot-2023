package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ArmInputs {
    public double shoulderAngle = 0;
    public double elbowAngle = 0;
    public double shoulderMotorPower = 0;
    public double elbowMotorPower = 0;
    public double shoulderSetpoint = 0;
    public double elbowSetpoint = 0;
    public double shoulderEncoderPosition = 0;
    public double elbowEncoderPosition = 0;
}