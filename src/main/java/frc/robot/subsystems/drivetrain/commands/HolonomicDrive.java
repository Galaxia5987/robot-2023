package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class HolonomicDrive extends CommandBase {
    protected final SwerveDrive swerveDrive = Robot.swerveSubsystem;
    protected final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    protected final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);
    protected final DoubleSupplier forward;
    protected final DoubleSupplier strafe;
    protected final DoubleSupplier rotation;

    public HolonomicDrive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        swerveDrive.drive(new DriveSignal(speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond, new Translation2d(), true));
    }

    protected ChassisSpeeds calculateVelocities() {
        double forwardVal = Utils.deadband(forwardLimiter.calculate(forward.getAsDouble()), 0.1);
        double strafeVal = Utils.deadband(strafeLimiter.calculate(strafe.getAsDouble()), 0.1);
        double rotationVal = Utils.deadband(rotationLimiter.calculate(rotation.getAsDouble()), 0.1);

        return new ChassisSpeeds(forwardVal * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                strafeVal * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }

    protected void turnToTarget(ChassisSpeeds speeds) {
        // TODO: Implement vision pipeline logic call to adjust to target
    }
}