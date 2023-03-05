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
    public double shoulderVelocity = 0;
    public double elbowVelocity = 0;
    public double[] armPosition = new double[2];
    public double[] inverseKinematicsSolution = new double[2];
    public double[] feedforward = new double[2];
    public double shoulderAcceleration = 0;
    public double elbowAcceleration = 0;
    public double shoulderError = 0;
    public double elbowError = 0;
    public double shoulderOutputVoltage = 0;
    public double[] finalSetpointAngles = new double[2];
    public double ySetpoint = 0;
}