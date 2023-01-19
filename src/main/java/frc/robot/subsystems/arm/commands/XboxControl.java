package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.PrototypeArm;

public class XboxControl extends CommandBase {
    private final PrototypeArm prototypeArm;
    private final XboxController xboxController;

    public XboxControl(PrototypeArm prototypeArm, XboxController xboxController) {
        this.prototypeArm = prototypeArm;
        this.xboxController = xboxController;
        addRequirements(prototypeArm);
    }

    @Override
    public void execute() {
        prototypeArm.setShoulderJointPower(prototypeArm.deadBend(xboxController.getLeftY()));
        prototypeArm.setElbowJointPower(prototypeArm.deadBend(xboxController.getRightY()));
    }

    @Override
    public void end(boolean interrupted) {
        prototypeArm.setShoulderJointPower(0);
        prototypeArm.setElbowJointPower(0);
    }
}
