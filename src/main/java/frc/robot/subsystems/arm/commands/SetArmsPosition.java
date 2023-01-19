package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmKinematics;
import frc.robot.subsystems.arm.ArmSystemModel;
import frc.robot.subsystems.arm.PrototypeArm;

public class SetArmsPosition extends CommandBase {
    private final PrototypeArm prototypeArm;
    private final Translation2d position;

    public SetArmsPosition(Translation2d position) {
        this.prototypeArm = new PrototypeArm();
        this.position = position;
        addRequirements();
    }

    @Override
    public void execute() {
        prototypeArm.setPosition(position);
    }
}
