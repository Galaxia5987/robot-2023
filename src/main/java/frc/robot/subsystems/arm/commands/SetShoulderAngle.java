package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class SetShoulderAngle extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final double angle;

    public SetShoulderAngle(double angle) {
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderJointAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.applyDeadband(angle - arm.getShoulderJointAngle(), ArmConstants.SETPOINT_DEADBAND) == 0;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
