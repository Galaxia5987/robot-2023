package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.autonomous.AutonomousLogInputs;
import frc.robot.autonomous.HolonomicFeedforward;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFConstants;
import frc.robot.utils.controllers.PIDFController;
import frc.robot.utils.ui.ButtonMap;
import org.littletonrobotics.junction.Logger;

import java.nio.file.Path;

public class HolonomicDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Gyroscope gyroscope;
    private final Limelight limelight;
    private final ButtonMap buttonMap;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);
    private final HolonomicFeedforward adjustFeedforward;
    private Timer timer = new Timer();
    private final PIDController adjustForwardController;
    private final PIDController adjustStrafeController;
    private final PIDFController adjustRotationController;
    private PathPlannerTrajectory trajectory;
    private final DriveSignal signal = new DriveSignal(0, 0, 0, new Translation2d(), true);
    private boolean lastScoreGamePiece = false;
    private final AutonomousLogInputs logInputs = new AutonomousLogInputs();

    public HolonomicDrive(SwerveDrive swerveDrive, Gyroscope gyroscope, Limelight limelight, ButtonMap buttonMap,
                          PIDConstants translationConstants, PIDFConstants rotationConstants, HolonomicFeedforward adjustFeedforward) {
        this.swerveDrive = swerveDrive;
        this.gyroscope = gyroscope;
        this.limelight = limelight;
        this.buttonMap = buttonMap;
        this.adjustForwardController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.adjustStrafeController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.adjustRotationController = new PIDFController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.kF);
        this.adjustFeedforward = adjustFeedforward;
        addRequirements(swerveDrive);

        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        var currentPose = swerveDrive.getPose();
        var currVelocity = swerveDrive.getSpeeds();
        // TODO: Fill these in
        if (buttonMap.getDoubleSubstation()) {

        } else if (buttonMap.getChargeStation()) {

        } else if (buttonMap.getIntakeCone()) {

        } else if (buttonMap.getIntakeCube()) {

        } else if (buttonMap.getScoreGamePiece()) {
            var botPose = limelight.getBotPose();
            boolean initialize = !lastScoreGamePiece && buttonMap.getScoreGamePiece();
            if (botPose.isPresent()) {
                if (initialize) {
                    gyroscope.resetYaw(botPose.get().getRotation());
                    swerveDrive.resetOdometry(botPose.get(), gyroscope.getYaw());

                    timer = new Timer();
                    timer.start();
                    timer.reset();
                    var pStart = new PathPoint(
                            currentPose.getTranslation(),
                            new Rotation2d(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond).plus(Rotation2d.fromDegrees(180)),
                            currentPose.getRotation(),
                            Math.hypot(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond));
                    var pEnd = new PathPoint(
                            VisionConstants.CUBE_ID1_POSE,
                            Rotation2d.fromDegrees(0),
                            Rotation2d.fromDegrees(0),
                            0);
                    trajectory = PathPlanner.generatePath(new PathConstraints(2, 1), false,
                            pStart, pEnd);
                }
                if (trajectory != null) {
                    adjustToTarget();
                }
            } else {
                signal.vx = 0;
                signal.vy = 0;
                signal.omega = 0;
                signal.fieldOriented = true;
                signal.centerOfRotation = new Translation2d();
            }
            System.out.println(botPose.isPresent());
        } else if (buttonMap.getSingleSubstation()) {

        } else if (buttonMap.lock()) {
            swerveDrive.lock();
            signal.vx = 0;
            signal.vy = 0;
            signal.omega = 0;
        } else {
            buttonMap.defaultDriveSignal(signal,
                    forwardLimiter, strafeLimiter, rotationLimiter);
            swerveDrive.drive(signal);
        }
        lastScoreGamePiece = buttonMap.getScoreGamePiece();
    }

    public void adjustToTarget() {
        double time = timer.get();
        var desiredState = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(time);
        var toDesiredTranslation = desiredState.poseMeters.minus(swerveDrive.getPose());
        Rotation2d heading = new Rotation2d(toDesiredTranslation.getX(), toDesiredTranslation.getY());

        Translation2d segment = new Translation2d(
                heading.getCos(), heading.getSin());

        Translation2d segmentVelocity = segment.times(desiredState.velocityMetersPerSecond);
        Translation2d segmentAcceleration = segment.times(desiredState.accelerationMetersPerSecondSq);

        Translation2d feedforwardResult = adjustFeedforward.calculateFeedforward(
                segmentVelocity, segmentAcceleration);

        var signal = new DriveSignal(
                adjustForwardController.calculate(swerveDrive.getPose().getTranslation().getX(), desiredState.poseMeters.getX())
                        + feedforwardResult.getX(),
                adjustStrafeController.calculate(swerveDrive.getPose().getTranslation().getY(), desiredState.poseMeters.getY())
                        + feedforwardResult.getY(),
                adjustRotationController.calculate(gyroscope.getYaw().getRadians(), desiredState.holonomicRotation.getRadians()),
                new Translation2d(),
                true
        );

        logInputs.desiredState = desiredState;
        logInputs.desiredSpeeds = signal.speeds();
        logInputs.time = time;

        Logger.getInstance().processInputs("Autonomous Path", logInputs);
        swerveDrive.drive(signal);
    }
}
