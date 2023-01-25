package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Gripper extends LoggedSubsystem<GripperLoggedInputs> {
    private final Solenoid innerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.OUTER_SOLENOID);
    private final Solenoid outerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.INNER_SOLENOID);


    public Gripper(GripperLoggedInputs inputs) {
        super(new GripperLoggedInputs());
    }

    /**
     * open the Gripper
     */

    public void open() {
        innerSolenoid.set(true);
        outerSolenoid.set(true);

    }

    /**
     * close the Gripper
     */
    public void close() {
        innerSolenoid.set(false);
        outerSolenoid.set(false);

    }

    /**
     * changes the current state to the other state
     */

    public void toggle() {
        innerSolenoid.toggle();
        outerSolenoid.toggle();

    }

    /**
     * return the current state of the solenoids
     */
    public boolean getOuterSolenoid() {
        return outerSolenoid.get();
    }

    public boolean getInnerSolenoid() {
        return innerSolenoid.get();
    }

    /**
     * update the logger inputs
     */

    @Override
    public void updateInputs() {
        loggerInputs.outerSolenoidState = innerSolenoid.get();

        loggerInputs.innerSolenoidState = outerSolenoid.get();

    }

    @Override
    public String getSubsystemName() {
        return "Gripper";
    }
}
