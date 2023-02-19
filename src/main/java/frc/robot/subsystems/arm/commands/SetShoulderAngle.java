package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.utils.Utils;
import frc.robot.utils.math.AngleUtil;

public class SetShoulderAngle extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final double angle;
    private final Timer timer = new Timer();
    private TrapezoidProfile profile;

    public SetShoulderAngle(double angle) {
        this.angle = AngleUtil.normalize(angle);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        double startAngle = AngleUtil.normalize(arm.getShoulderJointAngle().getDegrees());
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(45, 90),
                new TrapezoidProfile.State(angle, 0),
                new TrapezoidProfile.State(startAngle, 0));
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        var state = profile.calculate(timer.get());
        arm.setShoulderJointAngle(state.position);
    }

    @Override
    public boolean isFinished() {
        return Utils.epsilonEquals(angle, arm.getShoulderJointAngle().getDegrees(), ArmConstants.SETPOINT_DEADBAND);
    }
}
