package frc.robot.subsystems.gripper;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Gripper extends LoggedSubsystem<GripperLoggedInputs> {
    private final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.LEFT_SOLENOID);
    private final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.RIGHT_SOLENOID);



    public Gripper(GripperLoggedInputs inputs) {
        super(inputs);
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
    public void getRightSolenoid() {
        rightSolenoid.get();
    }

    public void getLeftSolenoid() {
        leftSolenoid.get();
    }

    /**
     * update the logger inputs
     */

    @Override
    public void updateInputs() {
        loggerInputs.leftSolenoid = leftSolenoid.get();

        loggerInputs.rightSolenoid = rightSolenoid.get();

    }

    @Override
    public String getSubsystemName() {
        return "Gripper";
    }
}
