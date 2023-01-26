package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Gripper extends LoggedSubsystem<GripperLoggedInputs> {
    private final Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID_1);
    private final Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.SOLENOID_2);

    public static Gripper INSTANCE;

    public static Gripper getInstance() {
        if (INSTANCE == null){
            INSTANCE = new Gripper(new GripperLoggedInputs());
        }
        return INSTANCE;
    }

    private Gripper(GripperLoggedInputs inputs) {
        super(new GripperLoggedInputs());
    }

    /**
     * open the Gripper
     */

    public void open() {
        solenoid1.set(true);
        solenoid2.set(true);

    }

    /**
     * close the Gripper
     */
    public void close() {
        solenoid1.set(false);
        solenoid2.set(false);

    }

    /**
     * changes the current state to the other state
     */

    public void toggle() {
        solenoid1.toggle();
        solenoid2.toggle();

    }

    /**
     * return the current state of the solenoids
     */
    public boolean getSolenoid2() {
        return solenoid2.get();
    }

    public boolean getSolenoid1() {
        return solenoid1.get();
    }

    /**
     * update the logger inputs
     */

    @Override
    public void updateInputs() {
        loggerInputs.firstSolenoidState = solenoid1.get();

        loggerInputs.secondSolenoidState = solenoid2.get();

    }

    @Override
    public String getSubsystemName() {
        return "Gripper";
    }
}
