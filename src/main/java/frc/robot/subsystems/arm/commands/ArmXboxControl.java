package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmXboxControl extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final XboxController xboxController;

    public ArmXboxControl(XboxController xboxController) {
        this.xboxController = xboxController;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double powerSho = MathUtil.applyDeadband(-xboxController.getLeftY(), 0.2);
        double powerEl = MathUtil.applyDeadband(-xboxController.getRightY(), 0.2);
        arm.setShoulderJointPower(powerSho);
        arm.setElbowJointPower(powerEl);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
