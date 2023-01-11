package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HolonomicDrive extends CommandBase {
    protected final SwerveDrive swerveDrive = Robot.swerveSubsystem;
    protected final PIDFController adjustController = new PIDFController(
            Constants.SwerveDrive.TARGET_ADJUST_Kp,
            0, 0,
            Constants.SwerveDrive.TARGET_ADJUST_Kf) {{
        enableContinuousInput(-180, 180);
    }};
    protected final SlewRateLimiter forwardLimiter = new SlewRateLimiter(Constants.SwerveDrive.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.SwerveDrive.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.SwerveDrive.ROTATION_SLEW_RATE_LIMIT);
    protected final DoubleSupplier forward;
    protected final DoubleSupplier strafe;
    protected final DoubleSupplier rotation;
    protected final BooleanSupplier turnToTarget;
    protected final BooleanSupplier lock;
    protected final BooleanSupplier robotOriented;

    public HolonomicDrive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation,
                          BooleanSupplier turnToTarget, BooleanSupplier lock, BooleanSupplier robotOriented) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.turnToTarget = turnToTarget;
        this.lock = lock;
        this.robotOriented = robotOriented;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        if (turnToTarget.getAsBoolean()) {
            turnToTarget(speeds);
        } else if (lock.getAsBoolean()) {
            swerveDrive.lock();
        } else {
            swerveDrive.drive(speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond, new Translation2d(), !robotOriented.getAsBoolean());
        }
    }

    protected ChassisSpeeds calculateVelocities() {
        double forwardVal = Utils.deadband(forwardLimiter.calculate(forward.getAsDouble()), 0.1);
        double strafeVal = Utils.deadband(strafeLimiter.calculate(strafe.getAsDouble()), 0.1);
        double rotationVal = Utils.deadband(rotationLimiter.calculate(rotation.getAsDouble()), 0.1);

        return new ChassisSpeeds(forwardVal * Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
                strafeVal * Constants.SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
                rotationVal * Constants.SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }

    protected void turnToTarget(ChassisSpeeds speeds) {
//        double rotationVal = adjustController.calculate(IntegratedUtils.angleToTarget(), 0);
//        swerveDrive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationVal);
        // TODO: Implement vision pipeline logic call to adjust to target
    }

    protected double smooth(double val) {
        return Math.signum(val) * Math.pow(Math.abs(val), Constants.SwerveDrive.SMOOTHING_FACTOR);
    }
}
