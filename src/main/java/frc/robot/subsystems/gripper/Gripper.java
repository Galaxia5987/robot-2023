package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Gripper extends SubsystemBase {
    private final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.LEFT_SOLENOID);
    private final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Gripper.RIGHT_SOLENOID);

   private Gripper(){
       leftSolenoid.set(false);
       rightSolenoid.set(false);
   }
    public void setSolenoid(boolean state) {
        leftSolenoid.set(state);
        rightSolenoid.set(state);

    }

    public void toggleSolenoid() {
        leftSolenoid.toggle();
        rightSolenoid.toggle();

    }

    public void getSolenoid() {
        rightSolenoid.get();
        leftSolenoid.get();

    }
}
