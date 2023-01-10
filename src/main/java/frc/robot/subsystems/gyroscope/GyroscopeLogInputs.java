package frc.robot.subsystems.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroscopeLogInputs implements LoggableInputs {
    public Rotation2d angle = Rotation2d.fromDegrees(0);
    public Rotation2d rawAngle = Rotation2d.fromDegrees(0);
    public Rotation2d zeroAngle = Rotation2d.fromDegrees(0);

    public GyroscopeLogInputs() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Angle", angle.getDegrees());
        table.put("RawAngle", rawAngle.getDegrees());
        table.put("ZeroAngle", zeroAngle.getDegrees());
    }

    @Override
    public void fromLog(LogTable table) {
        angle = Rotation2d.fromDegrees(table.getDouble("Angle", angle.getDegrees()));
        rawAngle = Rotation2d.fromDegrees(table.getDouble("RawAngle", rawAngle.getDegrees()));
        zeroAngle = Rotation2d.fromDegrees(table.getDouble("ZeroAngle", zeroAngle.getDegrees()));
    }
}
