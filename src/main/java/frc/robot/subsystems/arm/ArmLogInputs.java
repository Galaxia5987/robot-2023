package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmLogInputs implements LoggableInputs {
    public double elbowAngle = 0;
    public double shoulderAngle = 8;
    public double elbowMotorPower = 0;
    public double shoulderMotorPower = 0;
    public double shoulderSetPoint = 0;
    public double elbowSetPoint = 0;

    public ArmLogInputs() {
    }

    public void toLog(LogTable table) {
        table.put("elbowAngle", elbowAngle);
        table.put("shoulderAngle", shoulderAngle);
        table.put("elbowMotorPower", elbowMotorPower);
        table.put("shoulderMotorPower", shoulderMotorPower);
        table.put("shoulderSetPoint", shoulderSetPoint);
        table.put("elbowSetPoint", elbowSetPoint);
    }

    public void fromLog(LogTable table) {
        shoulderAngle = table.getDouble("shoulderAngle", shoulderAngle);
        elbowAngle = table.getDouble("elbowAngle", elbowAngle);
        shoulderMotorPower = table.getDouble("shoulderMotorPower", shoulderMotorPower);
        elbowMotorPower = table.getDouble("elbowMotorPower", elbowMotorPower);
        shoulderSetPoint = table.getDouble("shoulderSetPoint", shoulderSetPoint);
        elbowSetPoint = table.getDouble("elbowSetPoint", elbowSetPoint);
    }
}
