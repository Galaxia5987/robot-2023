package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

import java.util.function.Supplier;

public class SetArmsPosition extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Supplier<Translation2d> position;

    public SetArmsPosition(Translation2d position) {
        this(() -> position);
    }

    public SetArmsPosition(Supplier<Translation2d> position) {
        this.position = position;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setEndPosition(position.get());
    }

    @Override
    public boolean isFinished() {
        var position = this.position.get();
        var difference = arm.getEndPosition().minus(position);
        return MathUtil.applyDeadband(difference.getX(), ArmConstants.SETPOINT_DEADBAND) == 0 &&
                MathUtil.applyDeadband(difference.getY(), ArmConstants.SETPOINT_DEADBAND) == 0;
    }
}
