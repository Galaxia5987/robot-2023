package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.PrototypeArm;

public class SetArmsPosition extends CommandBase {
    private final PrototypeArm prototypeArm;
    private final Translation2d position;

    public SetArmsPosition(PrototypeArm prototypeArm, Translation2d position) {
        this.prototypeArm = prototypeArm;
        this.position = position;
        addRequirements();
    }

    @Override
    public void execute() {
        prototypeArm.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        prototypeArm.setShoulderJointPower(0);
        prototypeArm.setElbowJointPower(0);
    }
}
