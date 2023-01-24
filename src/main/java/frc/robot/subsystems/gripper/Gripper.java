package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Gripper extends LoggedSubsystem<GripperLoggedInputs> {
    private final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.LEFT_SOLENOID);
    private final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.RIGHT_SOLENOID);


    public Gripper(GripperLoggedInputs inputs) {
        super(new GripperLoggedInputs());
    }

    /**
     * open the Gripper
     */

    public void open() {
        leftSolenoid.set(true);
        rightSolenoid.set(true);

    }

    /**
     * close the Gripper
     */
    public void close() {
        leftSolenoid.set(false);
        rightSolenoid.set(false);

    }

    /**
     * changes the current state to the other state
     */

    public void toggle() {
        leftSolenoid.toggle();
        rightSolenoid.toggle();

    }

    /**
     * return the current state of the solenoids
     */
    public boolean getRightSolenoid() {
        return rightSolenoid.get();
    }

    public boolean getLeftSolenoid() {
        return leftSolenoid.get();
    }

    /**
     * update the logger inputs
     */

    @Override
    public void updateInputs() {
        loggerInputs.leftSolenoidState = leftSolenoid.get();

        loggerInputs.rightSolenoidState = rightSolenoid.get();

    }

    @Override
    public String getSubsystemName() {
        return "Gripper";
    }
}
