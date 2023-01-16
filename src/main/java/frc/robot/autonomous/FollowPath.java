package frc.robot.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import org.littletonrobotics.junction.Logger;

public class FollowPath extends CommandBase {
    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private final ProfiledPIDController forwardController;
    private final ProfiledPIDController strafeController;
    private final PIDController rotationController;
    private final HolonomicFeedforward feedforward;

    private final SwerveDrive swerveDrive;
    private final Gyroscope gyroscope;
    private final AutonomousLogInputs logInputs = new AutonomousLogInputs();

    public FollowPath(SwerveDrive swerveDrive, Gyroscope gyroscope, String trajectoryName, PIDConstants translationConstants, PIDConstants rotationConstants,
                      HolonomicFeedforward feedforward, double maxVelocity, double maxAcceleration) {
        this.swerveDrive = swerveDrive;
        this.gyroscope = gyroscope;
        this.trajectory = PathPlanner.loadPath(trajectoryName, maxVelocity, maxAcceleration);
        this.forwardController = new ProfiledPIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        this.strafeController = new ProfiledPIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        this.rotationController = new PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD);
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

        swerveDrive.resetOdometry(trajectory.getInitialPose(), gyroscope.getYaw());
        gyroscope.resetYaw(trajectory.getInitialState().holonomicRotation);
    }

    @Override
    public void execute() {
        var desiredState = trajectory.sample(timer.get());
        var currentPose = swerveDrive.getPose();
        var toDesiredTranslation = desiredState.poseMeters.minus(currentPose);
        Rotation2d heading = new Rotation2d(toDesiredTranslation.getX(), toDesiredTranslation.getY());
//        System.out.println("Heading: " + heading);

        Translation2d segment = new Translation2d(
                heading.getCos(), heading.getSin());
//        System.out.println("Segment: " + segment);

        Translation2d segmentVelocity = segment.times(desiredState.velocityMetersPerSecond);
        Translation2d segmentAcceleration = segment.times(desiredState.accelerationMetersPerSecondSq);

        Translation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);
//        System.out.println("FeedForward: " + feedforwardVector);

        var signal = new DriveSignal(
                -forwardController.calculate(currentPose.getTranslation().getX(), desiredState.poseMeters.getX())
                        - feedforwardVector.getX(),
                -strafeController.calculate(currentPose.getTranslation().getY(), desiredState.poseMeters.getY())
                        - feedforwardVector.getY(),
                -rotationController.calculate(gyroscope.getYaw().getRadians(), desiredState.poseMeters.getRotation().getRadians()),
                new Translation2d(),
                true
        );
        swerveDrive.drive(signal);

        logInputs.desiredState = desiredState;
        logInputs.desiredSpeeds = signal.speeds();
        logInputs.time = timer.get();

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