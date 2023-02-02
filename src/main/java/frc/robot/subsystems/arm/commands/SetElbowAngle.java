package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class SetElbowAngle extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final double angle;

    public SetElbowAngle(double angle) {
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setElbowJointAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.applyDeadband(angle - arm.getElbowJointAngle(), ArmConstants.SETPOINT_DEADBAND) == 0;
    }
}
