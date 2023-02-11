package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class JoystickDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);

    public JoystickDrive(SwerveDrive swerveDrive, Joystick leftJoystick, Joystick rightJoystick) {
        this.swerveDrive = swerveDrive;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        if (leftJoystick.getRawButton(Ports.UI.JOYSTICK_TOP_RIGHT_BUTTON)) {
            swerveDrive.lock();
        } else {
            double vx = -leftJoystick.getY();
            double vy = -leftJoystick.getX();
            double omega = -rightJoystick.getX();

            vx = forwardLimiter.calculate(vx);
            vy = strafeLimiter.calculate(vy);
            omega = rotationLimiter.calculate(omega);

            double magnitude = Math.hypot(vx, vy);
            double angle = Math.atan2(vy, vx);
            magnitude = MathUtil.applyDeadband(magnitude, 0.1);
            vx = Math.cos(angle) * magnitude;
            vy = Math.sin(angle) * magnitude;
            omega = MathUtil.applyDeadband(omega, 0.1);

            DriveSignal signal = new DriveSignal(
                    vx * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    vy * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    omega * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    new Translation2d(),
                    !leftJoystick.getTop());
            swerveDrive.drive(signal);
        }
    }
}
