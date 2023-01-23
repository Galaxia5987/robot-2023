package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PrototypeArmLogInputs implements LoggableInputs {
    public double elbowAngle = 0;
    public double shoulderAngle = 8;
    public double elbowMotorPower = 0;
    public double shoulderMotorPower = 0;
    public double shoulderP = 0;
    public double shoulderI = 0;
    public double shoulderD = 0;
    public double elbowP = 0;
    public double elbowI = 0;
    public double elbowD = 0;

    public PrototypeArmLogInputs() {
    }

    public void toLog(LogTable table) {
        table.put("elbowAngle", elbowAngle);
        table.put("shoulderAngle", shoulderAngle);
        table.put("elbowMotorPower", elbowMotorPower);
        table.put("shoulderMotorPower", shoulderMotorPower);
        table.put("shoulderP", shoulderP);
        table.put("shoulderI", shoulderI);
        table.put("shoulderD", shoulderD);
        table.put("elbowP", elbowP);
        table.put("elbowI", elbowI);
        table.put("elbowD", elbowD);
    }

    public void fromLog(LogTable table) {
        elbowAngle = table.getDouble("elbowAngle", elbowAngle);
        shoulderAngle = table.getDouble("shoulderAngle", shoulderAngle);
        elbowMotorPower = table.getDouble("elbowMotorPower", elbowMotorPower);
        shoulderMotorPower = table.getDouble("shoulderMotorPower", shoulderMotorPower);
        shoulderP = table.getDouble("shoulderP", shoulderP);
        shoulderI = table.getDouble("shoulderI", shoulderI);
        shoulderD = table.getDouble("shoulderD", shoulderD);
        elbowP = table.getDouble("elbowP", elbowP);
        elbowI = table.getDouble("elbowI", elbowI);
        elbowD = table.getDouble("elbowD", elbowD);
    }
}
