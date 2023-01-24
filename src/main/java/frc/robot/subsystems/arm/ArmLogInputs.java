package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmLogInputs implements LoggableInputs {
    public double elbowAngle = 0;
    public double shoulderAngle = 8;
    public double elbowMotorPower = 0;
    public double shoulderMotorPower = 0;

    public ArmLogInputs() {
    }

    public void toLog(LogTable table) {
        table.put("elbowAngle", elbowAngle);
        table.put("shoulderAngle", shoulderAngle);
        table.put("elbowMotorPower", elbowMotorPower);
        table.put("shoulderMotorPower", shoulderMotorPower);
    }

    public void fromLog(LogTable table) {
        elbowAngle = table.getDouble("elbowAngle", elbowAngle);
        shoulderAngle = table.getDouble("shoulderAngle", shoulderAngle);
        elbowMotorPower = table.getDouble("elbowMotorPower", elbowMotorPower);
        shoulderMotorPower = table.getDouble("shoulderMotorPower", shoulderMotorPower);
    }
}
