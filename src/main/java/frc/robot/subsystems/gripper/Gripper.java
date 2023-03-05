package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Gripper extends LoggedSubsystem<GripperLoggedInputs> {
    private static Gripper INSTANCE;
    private final AnalogInput distance = new AnalogInput(0);
    private final Solenoid gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID);

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


    @Override
    public String getSubsystemName() {
        return "Gripper";
    }

    @Override
    public void updateInputs() {
        loggerInputs.isOpen = isOpen();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance sensor", distance.getValue());
    }
}
