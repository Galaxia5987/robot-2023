package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class SetShoulderAngle extends CommandBase {
    private final Arm arm;
    private final double angle;

    public SetShoulderAngle(double angle) {
        this.arm = Arm.getInstance();
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderJointAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.applyDeadband(angle, ArmConstants.SETPOINT_DEADBAND) == arm.getShoulderJointAngle();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
