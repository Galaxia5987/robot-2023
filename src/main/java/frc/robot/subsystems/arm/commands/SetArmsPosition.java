package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

import java.util.function.Function;
import java.util.function.Supplier;

public class SetArmsPosition extends CommandBase {
    private final Arm arm;
    private final Supplier<Translation2d> position;

    public SetArmsPosition(Translation2d position) {
        this(() -> position);
    }

    public SetArmsPosition(Supplier<Translation2d> position) {
        this.arm = Arm.getInstance();
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
        return MathUtil.applyDeadband(position.getX() - arm.getEndPosition().getX(), ArmConstants.SETPOINT_DEADBAND) == 0 &&
                MathUtil.applyDeadband(position.getY() - arm.getEndPosition().getY(), ArmConstants.SETPOINT_DEADBAND) == 0;
    }
}
