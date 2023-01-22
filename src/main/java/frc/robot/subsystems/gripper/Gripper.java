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
    private GripperLoggedInputs gripperLoggedInputs;
    private final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.LEFT_SOLENOID);
    private final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.RIGHT_SOLENOID);

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("gripper");


    public Gripper(GripperLoggedInputs inputs) {
        super(inputs);
    }

    /**
     * setSolenoid sets the Solenoid state
     * @param state
     */

    public void setSolenoid(boolean state) {
        leftSolenoid.set(state);
        rightSolenoid.set(state);

    }

    /**
     * changes the current state to the other state
     */

    public void toggleSolenoid() {
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
        gripperLoggedInputs.leftSolenoid = leftSolenoid.get();

        gripperLoggedInputs.rightSolenoid = rightSolenoid.get();

    }

    @Override
    public String getSubsystemName() {
        return "Gripper";
    }
}
