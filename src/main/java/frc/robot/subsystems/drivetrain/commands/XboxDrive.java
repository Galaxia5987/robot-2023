package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFController;

public class XboxDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final XboxController xboxController;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);

    public XboxDrive(SwerveDrive swerveDrive, XboxController xboxController) {
        this.swerveDrive = swerveDrive;
        this.xboxController = xboxController;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double vx = -xboxController.getLeftY();
        double vy = -xboxController.getLeftX();
        double omega = -xboxController.getRightX();

        vx = forwardLimiter.calculate(vx);
        vy = strafeLimiter.calculate(vy);
        omega = rotationLimiter.calculate(omega);

        double magnitude = Math.hypot(vx, vy);
        double angle = Math.atan2(vy, vx);
        magnitude = MathUtil.applyDeadband(magnitude, 0.2);
        vx = Math.cos(angle) * magnitude;
        vy = Math.sin(angle) * magnitude;
        omega = MathUtil.applyDeadband(omega, 0.2);

        DriveSignal signal = new DriveSignal(
                vx * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                vy * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                omega * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                new Translation2d(),
                true);
        swerveDrive.drive(signal);
    }
}
