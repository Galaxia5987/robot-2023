package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
public class ArmInputs {
    public double elbowAngle = 0;
    public double shoulderAngle = 8;
    public double elbowMotorPower = 0;
    public double shoulderMotorPower = 0;
    public double shoulderSetpoint = 0;
    public double elbowSetpoint = 0;
}
