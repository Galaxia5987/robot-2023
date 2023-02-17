package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.utils.Utils;
import org.littletonrobotics.junction.Logger;

public class SetArmsPositionAngular extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private double shoulderAngle;
    private double elbowAngle;

    private TrapezoidProfile shoulderProfile;
    private TrapezoidProfile elbowProfile;

    private final Timer timer = new Timer();

    public SetArmsPositionAngular(Translation2d position) {
        var solution = arm.getKinematics().inverseKinematics(position);
        shoulderAngle = Math.toDegrees(solution.shoulderAngle);
        elbowAngle = Math.toDegrees(solution.elbowAngle);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        double currentShoulderAngle = arm.getShoulderJointAngle().getDegrees();
        double currentElbowAngle = arm.getElbowJointAngle().getDegrees();
        shoulderProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(45, 90),
                new TrapezoidProfile.State(shoulderAngle, 0),
                new TrapezoidProfile.State(currentShoulderAngle, 0));
        elbowProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(180, 270),
                new TrapezoidProfile.State(elbowAngle, 0),
                new TrapezoidProfile.State(currentElbowAngle, 0));

        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double time = timer.get();
        double shoulder = shoulderProfile.calculate(time).position;
        double elbow = elbowProfile.calculate(time).position;

        arm.setShoulderJointAngle(shoulder);
        arm.setElbowJointAngle(elbow);

        Logger.getInstance().recordOutput("Arm Desired Shoulder Angle", shoulder);
        Logger.getInstance().recordOutput("Arm Desired Elbow Angle", elbow);
    }

    @Override
    public boolean isFinished() {
        return Utils.epsilonEquals(arm.getShoulderJointAngle().getDegrees(), shoulderAngle) &&
                Utils.epsilonEquals(arm.getElbowJointAngle().getDegrees(), elbowAngle);
    }
}
