package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Gripper extends LoggedSubsystem<GripperLoggedInputs> {
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID);

    public static Gripper INSTANCE;

    /**
     * @return the instance of the subsystem
     */
    public static Gripper getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Gripper(new GripperLoggedInputs());
        }
        return INSTANCE;
    }

    /**
     * constructor
     */
    private Gripper(GripperLoggedInputs inputs) {
        super(new GripperLoggedInputs());
    }

    /**
     * open the Gripper
     */

    public void open() {
        solenoid.set(true);

    }

    /**
     * close the Gripper
     */
    public void close() {
        solenoid.set(false);

    }

    /**
     * changes the current state to the other state
     */

    public void toggle() {
        solenoid.toggle();

    }

    /**
     * return the current state of the solenoids
     */

    public boolean getSolenoid() {
        return solenoid.get();
    }

    /**
     * update the logger inputs
     */

    @Override
    public void updateInputs() {
        loggerInputs.SolenoidState = solenoid.get();
    }

    @Override
    public String getSubsystemName() {
        return "Gripper";
    }
}
