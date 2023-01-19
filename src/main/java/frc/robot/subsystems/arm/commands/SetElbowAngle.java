package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.PrototypeArm;

public class SetElbowAngle extends CommandBase {
    private final PrototypeArm prototypeArm;
    private final double angle;

    public SetElbowAngle(double angle) {
        this.prototypeArm = new PrototypeArm();
        this.angle = angle;
        addRequirements(prototypeArm);
    }

    @Override
    public void execute() {
        prototypeArm.setElbowJointPosition(angle);
    }
}
