package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmXboxControl extends CommandBase {
    private final Arm arm;
    private final XboxController xboxController;

    public ArmXboxControl(XboxController xboxController) {
        this.arm = Arm.getInstance();
        this.xboxController = xboxController;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setShoulderJointPower(-xboxController.getLeftY());
        arm.setElbowJointPower(-xboxController.getRightY());
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
