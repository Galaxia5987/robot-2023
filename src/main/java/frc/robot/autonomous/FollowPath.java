package frc.robot.autonomous;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class FollowPath extends CommandBase {
    private final Trajectory trajectory;
    private final Timer timer = new Timer();
    private final PIDController forwardController;
    private final PIDController strafeController;
    private final PIDController rotationController;
    private final HolonomicFeedforward feedforward;

    private final SwerveDrive swerveDrive = Robot.swerveSubsystem;

    public FollowPath(Trajectory trajectory, PIDConstants translationConstants, PIDConstants rotationConstants,
                      HolonomicFeedforward feedforward) {
        this.trajectory = trajectory;
        this.forwardController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.strafeController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.rotationController = new PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD);
        this.rotationController.enableContinuousInput(0.0, 2.0 * Math.PI);

        this.feedforward = feedforward;

        swerveDrive.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double time = timer.get();
        var desiredState = trajectory.sample(time);
        var toDesiredTranslation = desiredState.poseMeters.minus(swerveDrive.getPose());
        Rotation2d heading = new Rotation2d(toDesiredTranslation.getX(), toDesiredTranslation.getY());
        Translation2d segment = new Translation2d(
            heading.getCos(), heading.getSin());

        Translation2d segmentVelocity = segment.times(desiredState.velocityMetersPerSecond);
        Translation2d segmentAcceleration = segment.times(desiredState.accelerationMetersPerSecondSq);

        Translation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        forwardController.setSetpoint(desiredState.poseMeters.getX());
        strafeController.setSetpoint(desiredState.poseMeters.getY());
        rotationController.setSetpoint(desiredState.poseMeters.getRotation().getRadians());

        swerveDrive.drive(new DriveSignal(
                forwardController.calculate(swerveDrive.getPose().getTranslation().getX()) + feedforwardVector.getX(),
                strafeController.calculate(swerveDrive.getPose().getTranslation().getY()) + feedforwardVector.getY(),
                rotationController.calculate(swerveDrive.getPose().getRotation().getRadians()),
                new Translation2d(),
                false
        ));
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