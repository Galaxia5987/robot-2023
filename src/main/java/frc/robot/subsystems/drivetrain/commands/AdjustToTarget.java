package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.server.PathPlannerServerThread;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.autonomous.AutonomousLogInputs;
import frc.robot.autonomous.HolonomicFeedforward;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFConstants;
import frc.robot.utils.controllers.PIDFController;
import org.littletonrobotics.junction.Logger;

public class AdjustToTarget extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Gyroscope gyroscope;
    private final Limelight vision;

    private final Timer timer = new Timer();
    private PathPlannerTrajectory trajectory;

    private final HolonomicFeedforward adjustFeedforward;
    private final PIDController adjustForwardController;
    private final PIDController adjustStrafeController;
    private final PIDFController adjustRotationController;
    private final AutonomousLogInputs logInputs = new AutonomousLogInputs();
    private Limelight.AprilTagTarget aprilTagTarget;

    private boolean noTargets = false;

    public AdjustToTarget(SwerveDrive swerveDrive, Gyroscope gyroscope, Limelight vision,
                          PIDConstants translationConstants, PIDFConstants rotationConstants, HolonomicFeedforward adjustFeedforward) {
        this.swerveDrive = swerveDrive;
        this.gyroscope = gyroscope;
        this.vision = vision;
        this.adjustForwardController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.adjustStrafeController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.adjustRotationController = new PIDFController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.kF);
        this.adjustFeedforward = adjustFeedforward;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        var aprilTag = vision.getAprilTagTarget();
        var currVelocity = swerveDrive.getSpeeds();

        if (aprilTag.isPresent()) {
            this.aprilTagTarget = aprilTag.get();

            swerveDrive.resetOdometry(new Pose2d(aprilTag.get().currentTranslation, new Rotation2d()), gyroscope.getYaw());

            timer.start();
            timer.reset();
            var pStart = new PathPoint(
                    aprilTag.get().currentTranslation,
                    new Rotation2d(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond).minus(aprilTagTarget.zeroHeading),
                    gyroscope.getYaw().minus(aprilTagTarget.zeroHeading),
                    Math.hypot(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond));
            var pEnd = new PathPoint(
                    aprilTag.get().desiredTranslation,
                    aprilTag.get().targetHeading,
                    aprilTag.get().targetYaw,
                    0);
            trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.generatePath(new PathConstraints(4, 2), false,
                    pStart, pEnd), DriverStation.getAlliance());
        } else {
            noTargets = true;
        }
    }

    @Override
    public void execute() {
        double time = timer.get();
        var desiredState = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(time);
        var toDesiredTranslation = desiredState.poseMeters.minus(swerveDrive.getPose());
        Rotation2d heading = new Rotation2d(toDesiredTranslation.getX(), toDesiredTranslation.getY())
                .minus(Rotation2d.fromDegrees(180));
        logInputs.heading = heading;

        Translation2d segment = new Translation2d(
                heading.getCos(), heading.getSin());

        Translation2d segmentVelocity = segment.times(desiredState.velocityMetersPerSecond);
        Translation2d segmentAcceleration = segment.times(desiredState.accelerationMetersPerSecondSq);

        Translation2d feedforwardResult = adjustFeedforward.calculateFeedforward(
                segmentVelocity, segmentAcceleration);

        var signal = new DriveSignal(
                adjustForwardController.calculate(swerveDrive.getPose().getTranslation().getX(), desiredState.poseMeters.getX()) * -1
                        + feedforwardResult.getX(),
                adjustStrafeController.calculate(swerveDrive.getPose().getTranslation().getY(), desiredState.poseMeters.getY())  * -1
                        + feedforwardResult.getY(),
                adjustRotationController.calculate(gyroscope.getYaw().getRadians(), desiredState.holonomicRotation.plus(aprilTagTarget.zeroHeading).getRadians()),
                new Translation2d(),
                true
        );

        logInputs.desiredState = desiredState.poseMeters;
        logInputs.desiredSpeeds = signal.speeds();
        logInputs.time = time;

        Logger.getInstance().processInputs("Autonomous Path", logInputs);
//        swerveDrive.drive(signal);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return (swerveDrive.getPose().minus(trajectory.getEndState().poseMeters).getTranslation().getNorm() < 0.1
                && Utils.epsilonEquals(swerveDrive.getPose().getRotation().getDegrees(), 0, 2))
                || noTargets;
    }
}
