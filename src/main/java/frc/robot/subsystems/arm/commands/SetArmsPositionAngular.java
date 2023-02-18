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
    private final double shoulderAngle;
    private final double elbowAngle;
    private final double deadBand;

    private TrapezoidProfile shoulderProfile;
    private TrapezoidProfile elbowProfile;

    private final Timer timer = new Timer();

    private final Translation2d position;

    private double maxShoulderVelocity = 180;
    private double maxShoulderAcceleration = 90;

    //    private double maxElbowVelocity = 70;
    private double maxElbowVelocity = 180;
    //    private double maxElbowAcceleration = 180;
    private double maxElbowAcceleration = 270;

    public SetArmsPositionAngular(Translation2d position, double deadBand) {
        this(position,
                deadBand,
                180, 270);
    }

    public SetArmsPositionAngular(Translation2d position, double deadBand, double maxElbowVelocity, double maxElbowAcceleration) {
        var solution = arm.getKinematics().inverseKinematics(position);
        this.shoulderAngle = Math.toDegrees(solution.shoulderAngle);
        this.elbowAngle = Math.toDegrees(solution.elbowAngle);
        this.deadBand = deadBand;
        this.maxElbowVelocity = maxElbowVelocity;
        this.maxElbowAcceleration = maxElbowAcceleration;
        this.position = position;
    }

    public SetArmsPositionAngular(Translation2d position, double maxElbowVelocity, double maxElbowAcceleration) {
        this(position,
                0.02,
                maxElbowVelocity,
                maxElbowAcceleration);
    }

    public SetArmsPositionAngular(Translation2d position) {
        this(position, 0.02);
    }

    public SetArmsPositionAngular(double shoulderAngle, double elbowAngle, double deadBand,
                                  double maxElbowVelocity, double maxElbowAcceleration) {
        this.shoulderAngle = shoulderAngle;
        this.elbowAngle = elbowAngle;
        this.deadBand = deadBand;
        this.maxElbowVelocity = maxElbowVelocity;
        this.maxElbowAcceleration = maxElbowAcceleration;
        this.position = arm.getKinematics().forwardKinematics(Math.toRadians(shoulderAngle), Math.toRadians(elbowAngle));
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        double currentShoulderAngle = arm.getShoulderJointAngle().getDegrees();
        double currentElbowAngle = arm.getElbowJointAngle().getDegrees();
        shoulderProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(180, 90),
                new TrapezoidProfile.State(shoulderAngle, 0),
                new TrapezoidProfile.State(currentShoulderAngle, 0));
        elbowProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(270, 110),
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
    }

    @Override
    public boolean isFinished() {
        return arm.getEndPosition().minus(position).getNorm() < deadBand;
//        return false;
    }
}
