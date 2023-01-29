package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class SetElbowAngle extends CommandBase {
    private final Arm arm;
    private final double angle;

    public SetElbowAngle(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setElbowJointAngle(angle);
    }

    @Override
    public boolean isFinished() {
        if (arm.getElbowJointAngle() == angle) {
            return true;
        }
        return false;
    }
}
