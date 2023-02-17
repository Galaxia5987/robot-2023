package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class SetArmsPositionLinear extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Translation2d position;

    private TrapezoidProfile xProfile;
    private TrapezoidProfile yProfile;

    private final Timer timer = new Timer();

    public SetArmsPositionLinear(Translation2d position) {
        this.position = position;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Translation2d currentPosition = arm.getEndPosition();
        xProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.2, 0.4),
                new TrapezoidProfile.State(position.getX(), 0),
                new TrapezoidProfile.State(currentPosition.getX(), 0));
        yProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.2, 0.4),
                new TrapezoidProfile.State(position.getY(), 0),
                new TrapezoidProfile.State(currentPosition.getY(), 0));

        timer.start();
        timer.reset();

        arm.setFinalSetpointAngles(position);
    }

    @Override
    public void execute() {
        double time = timer.get();
        double x = xProfile.calculate(time).position;
        double y = yProfile.calculate(time).position;

        arm.setEndPosition(new Translation2d(x, y));
        Logger.getInstance().recordOutput("Arm Desired X", x);
        Logger.getInstance().recordOutput("Arm Desired Y", y);
    }

    @Override
    public boolean isFinished() {
        var position = this.position;
        var difference = arm.getEndPosition().minus(position);
        return MathUtil.applyDeadband(difference.getX(), ArmConstants.SETPOINT_DEADBAND) == 0 &&
                MathUtil.applyDeadband(difference.getY(), ArmConstants.SETPOINT_DEADBAND) == 0;
    }
}
