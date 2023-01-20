package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.autonomous.HolonomicFeedforward;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.controllers.PIDFConstants;
import frc.robot.utils.controllers.PIDFController;
import frc.robot.utils.ui.ButtonMap;

public class HolonomicDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Gyroscope gyroscope;
    private final ButtonMap buttonMap;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);
    private final HolonomicFeedforward adjustFeedforward;
    private final Timer timer = new Timer();
    private final PIDController adjustForwardController;
    private final PIDController adjustStrafeController;
    private final PIDFController adjustRotationController;
    private TrapezoidProfile profileX;
    private TrapezoidProfile profileY;
    private double xLastVelocity = 0;
    private double yLastVelocity = 0;
    private double lastTime = 0;
    private final DriveSignal signal = new DriveSignal(0, 0, 0, new Translation2d(), true);

    public HolonomicDrive(SwerveDrive swerveDrive, Gyroscope gyroscope, ButtonMap buttonMap,
                          PIDConstants translationConstants, PIDFConstants rotationConstants, HolonomicFeedforward adjustFeedforward) {
        this.swerveDrive = swerveDrive;
        this.gyroscope = gyroscope;
        this.buttonMap = buttonMap;
        this.adjustForwardController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.adjustStrafeController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        this.adjustRotationController = new PIDFController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.kF);
        this.adjustFeedforward = adjustFeedforward;
        addRequirements(swerveDrive);

        timer.start();
    }

    @Override
    public void execute() {
        // TODO: Fill these in
        if (buttonMap.getDoubleSubstation()) {

        } else if (buttonMap.getChargeStation()) {

        } else if (buttonMap.getIntakeCone()) {

        } else if (buttonMap.getIntakeCube()) {

        } else if (buttonMap.getScoreGamePiece()) {

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
    }

    public void adjustToTarget(Translation2d desiredTranslation, Rotation2d yaw, Rotation2d desiredYaw, boolean initialize) {
        var currPose = swerveDrive.getPose();
        var currVelocity = swerveDrive.getSpeeds();
        if (initialize) {
            lastTime = 0;
            timer.reset();
            profileX = new TrapezoidProfile(new TrapezoidProfile.Constraints(4, 2),
                    new TrapezoidProfile.State(currPose.getX(), currVelocity.vxMetersPerSecond),
                    new TrapezoidProfile.State(desiredTranslation.getX(), 0));
            profileY = new TrapezoidProfile(new TrapezoidProfile.Constraints(4, 2),
                    new TrapezoidProfile.State(currPose.getY(), currVelocity.vyMetersPerSecond),
                    new TrapezoidProfile.State(desiredTranslation.getY(), 0));
        }
        double time = timer.get();
        double dt = time - lastTime;
        var xDesiredState = profileX.calculate(time);
        var yDesiredState = profileY.calculate(time);
        double xAcceleration = (xDesiredState.velocity - xLastVelocity) / dt;
        double yAcceleration = (yDesiredState.velocity - yLastVelocity) / dt;
        Translation2d error = new Translation2d(xDesiredState.position, yDesiredState.position).minus(currPose.getTranslation());
        Translation2d segment = error.div(error.getNorm());

        var feedForward = adjustFeedforward.calculateFeedforward(
                segment.times(Math.hypot(xDesiredState.velocity, yDesiredState.velocity)),
                segment.times(Math.hypot(xAcceleration, yAcceleration)));

        signal.vx = adjustForwardController.calculate(currPose.getX(), xDesiredState.position)
                + feedForward.getX();
        signal.vy = adjustStrafeController.calculate(currPose.getY(), yDesiredState.position)
                + feedForward.getY();
        signal.omega = adjustRotationController.calculate(desiredYaw.minus(yaw).getRadians());
        signal.centerOfRotation = new Translation2d();
        signal.fieldOriented = true;

        xLastVelocity = xDesiredState.velocity;
        yLastVelocity = yDesiredState.velocity;
        lastTime = time;
    }
}
