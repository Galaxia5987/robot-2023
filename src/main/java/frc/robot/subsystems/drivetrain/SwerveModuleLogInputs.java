package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleLogInputs implements LoggableInputs {
    public Rotation2d aAngle = Rotation2d.fromDegrees(0);
    public Rotation2d aSetpoint = Rotation2d.fromDegrees(0);
    public Rotation2d encoderAngle = Rotation2d.fromDegrees(0);
    public Rotation2d offsetAngle = Rotation2d.fromDegrees(0);
    public double aPosition = 0;
    public double aCurrent = 0;

    public double dVelocity = 0;
    public double dSetpoint = 0;
    public double dCurrent = 0;
    public double moduleDistance = 0;

    public SwerveModuleLogInputs() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("aAngle", aAngle.getDegrees());
        table.put("aSetpoint", aSetpoint.getDegrees());
        table.put("encoderAngle", encoderAngle.getDegrees());
        table.put("offsetAngle", offsetAngle.getDegrees());
        table.put("aPosition", aPosition);
        table.put("aCurrent", aCurrent);

        table.put("dVelocity", dVelocity);
        table.put("dSetpoint", dSetpoint);
        table.put("dCurrent", dCurrent);
        table.put("moduleDistance", moduleDistance);
    }

    @Override
    public void fromLog(LogTable table) {
        aAngle = Rotation2d.fromDegrees(table.getDouble("aAngle", aAngle.getDegrees()));
        aSetpoint = Rotation2d.fromDegrees(table.getDouble("aSetpoint", aSetpoint.getDegrees()));
        aPosition = table.getDouble("aPosition", aPosition);
        aCurrent = table.getDouble("aCurrent", aCurrent);

        dVelocity = table.getDouble("dVelocity", dVelocity);
        dCurrent = table.getDouble("dCurrent", dCurrent);
        moduleDistance = table.getDouble("moduleDistance", moduleDistance);
    }
}
