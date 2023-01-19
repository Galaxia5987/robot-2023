package frc.robot.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.controllers.PIDFConstants;
import frc.robot.utils.controllers.PIDFController;
import org.apache.velocity.runtime.parser.node.MathUtils;
import org.littletonrobotics.junction.Logger;

public class FollowPath extends CommandBase {
    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private PIDController forwardController;
    private PIDController strafeController;
    private PIDFController rotationController;
    private final HolonomicFeedforward feedforward;

    private final SwerveDrive swerveDrive;
    private final Gyroscope gyroscope;
    private final AutonomousLogInputs logInputs = new AutonomousLogInputs();

    public FollowPath(SwerveDrive swerveDrive, Gyroscope gyroscope, String trajectoryName, PIDConstants translationConstants, PIDFConstants rotationConstants,
                      HolonomicFeedforward feedforward, double maxVelocity, double maxAcceleration) {
        this.swerveDrive = swerveDrive;
        this.gyroscope = gyroscope;
        this.trajectory = PathPlanner.loadPath(trajectoryName, maxVelocity, maxAcceleration);
        this.forwardController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.strafeController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.rotationController = new PIDFController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.kF);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.feedforward = feedforward;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        logInputs.initialPose = trajectory.getInitialState();
        logInputs.finalPose = trajectory.getEndState();

        gyroscope.resetYaw(trajectory.getInitialState().holonomicRotation);
        swerveDrive.resetOdometry(trajectory.getInitialPose(), gyroscope.getYaw());

        forwardController = new PIDController(SmartDashboard.getNumber("Kp", 0),
                SmartDashboard.getNumber("Ki", 0),
                SmartDashboard.getNumber("Kd", 0));
        strafeController = new PIDController(SmartDashboard.getNumber("Kp", 0),
                SmartDashboard.getNumber("Ki", 0),
                SmartDashboard.getNumber("Kd", 0));
        rotationController = new PIDFController(SmartDashboard.getNumber("Rotation_Kp", 0),
                SmartDashboard.getNumber("Rotation_Ki", 0),
                SmartDashboard.getNumber("Rotation_Kd", 0),
                SmartDashboard.getNumber("Rotation_Kf", 0));
    }

    @Override
    public void execute() {
        double time = timer.get();
        var desiredState = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(time);
        var currentPose = swerveDrive.getPose();
        var toDesiredTranslation = desiredState.poseMeters.minus(currentPose);
        Rotation2d heading = new Rotation2d(toDesiredTranslation.getX(), toDesiredTranslation.getY());

        Translation2d segment = new Translation2d(
                heading.getCos(), heading.getSin());

        Translation2d segmentVelocity = segment.times(desiredState.velocityMetersPerSecond);
        Translation2d segmentAcceleration = segment.times(desiredState.accelerationMetersPerSecondSq);

        Translation2d feedforwardResult = feedforward.calculateFeedforward(
                segmentVelocity, segmentAcceleration);

        var signal = new DriveSignal(
                forwardController.calculate(currentPose.getTranslation().getX(), desiredState.poseMeters.getX())
                        + feedforwardResult.getX(),
                strafeController.calculate(currentPose.getTranslation().getY(), desiredState.poseMeters.getY())
                        + feedforwardResult.getY(),
                    rotationController.calculate(gyroscope.getYaw().getRadians(), desiredState.holonomicRotation.getRadians()),
                new Translation2d(),
                true
        );
        swerveDrive.drive(signal);

        logInputs.desiredState = desiredState;
        logInputs.desiredSpeeds = signal.speeds();
        logInputs.time = time;

        Logger.getInstance().processInputs("Autonomous Path", logInputs);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }
}