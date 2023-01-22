package frc.robot.subsystems.gripper.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gripper.Gripper;

public class toggleSolenoid extends CommandBase {
    private Gripper gripper;
    @Override
    public void initialize() {
        gripper.toggleSolenoid();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
