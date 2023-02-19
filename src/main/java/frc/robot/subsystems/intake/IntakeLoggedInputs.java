package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    public double velocity;
    public double power;
    public double anglePower;
    public double angle;
    public double current;

    /**
     * Implement the variables inside the table.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("power", power);
        table.put("angle", angle);
        table.put("velocity", velocity);
        table.put("current", current);
        table.put("anglePower", anglePower);
    }

    /**
     * Update the variables value from the logger.
     */
    @Override
    public void fromLog(LogTable table) {
        power = table.getDouble("power", power);
        angle = table.getDouble("angle", angle);
        velocity = table.getDouble("velocity", velocity);
    }
}
