package frc.robot.subsystems.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroscopeLogInputs implements LoggableInputs {
    public Rotation2d yaw = Rotation2d.fromDegrees(0);
    public Rotation2d rawYaw = Rotation2d.fromDegrees(0);
    public Rotation2d zeroYaw = Rotation2d.fromDegrees(0);
    public Rotation2d pitch = Rotation2d.fromDegrees(0);
    public Rotation2d roll = Rotation2d.fromDegrees(0);

    public GyroscopeLogInputs() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Yaw", yaw.getDegrees());
        table.put("RawYaw", rawYaw.getDegrees());
        table.put("ZeroYaw", zeroYaw.getDegrees());
        table.put("Pitch", pitch.getDegrees());
        table.put("Roll", roll.getDegrees());
    }

    @Override
    public void fromLog(LogTable table) {
        yaw = Rotation2d.fromDegrees(table.getDouble("Yaw", yaw.getDegrees()));
        rawYaw = Rotation2d.fromDegrees(table.getDouble("RawYaw", rawYaw.getDegrees()));
        zeroYaw = Rotation2d.fromDegrees(table.getDouble("ZeroYaw", zeroYaw.getDegrees()));
        pitch = Rotation2d.fromDegrees(table.getDouble("Pitch", pitch.getDegrees()));
        roll = Rotation2d.fromDegrees(table.getDouble("Roll", roll.getDegrees()));
    }
}
