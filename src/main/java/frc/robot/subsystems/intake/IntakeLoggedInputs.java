package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    public double motorPower;
    public double angleMotorAngle;

    /**
     * implement the variables inside the table
     */
    @Override
    public void toLog(LogTable table) {
        table.put("motor power", motorPower);
        table.put("angle motor power", angleMotorAngle);
    }

    /**
     * update the variables value from the logger
     */
    @Override
    public void fromLog(LogTable table) {
        motorPower = table.getDouble("motor power", motorPower);
        angleMotorAngle = table.getDouble("angle motor power", angleMotorAngle);
    }
}
