package frc.robot.subsystems.gripper;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Gripper extends LoggedSubsystem<GripperLoggedInputs> {
    private static Gripper INSTANCE;
    private final AnalogInput distance = new AnalogInput(0);
    private final Solenoid gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID);
    private final MedianFilter distanceFilter = new MedianFilter(3);

    private Gripper() {
        super(new GripperLoggedInputs());
    }

    /**
     * @return the instance of the subsystem.
     */
    public static Gripper getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Gripper();
        }
        return INSTANCE;
    }

    /**
     * Open the Gripper.
     */
    public void open() {
        gripperSolenoid.set(true);
    }

    /**
     * Close the Gripper.
     */
    public void close() {
        gripperSolenoid.set(false);
    }

    /**
     * Changes the current state to the other state.
     */
    public void toggle() {
        gripperSolenoid.toggle();
    }

    /**
     * @return the current state of the solenoids.
     */
    public boolean isOpen() {
        return gripperSolenoid.get();
    }

    public double getDistance() {
        return loggerInputs.distance;
    }


    @Override
    public String getSubsystemName() {
        return "Gripper";
    }

    @Override
    public void updateInputs() {
        loggerInputs.isOpen = isOpen();
        loggerInputs.distance = distanceFilter.calculate(4800 / (200 * distance.getVoltage() - 20.0));
    }
}
