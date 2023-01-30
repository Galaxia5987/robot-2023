package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveDriveLogInputs implements LoggableInputs {
    public double[] speeds = new double[]{0, 0, 0};
    public double[] setpoint = new double[]{0, 0, 0};
    public double[] pose = new double[]{0, 0, 0};

    public SwerveDriveLogInputs() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Speeds", speeds);
        table.put("Setpoint", setpoint);
        table.put("Pose", pose);
    }

    @Override
    public void fromLog(LogTable table) {
        speeds = table.getDoubleArray("Speeds", speeds);
        setpoint = table.getDoubleArray("Setpoint", setpoint);
        pose = table.getDoubleArray("Pose", pose);
    }
}