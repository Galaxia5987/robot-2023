package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.PIDFController;

public class JoystickDrive extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.XY_SLEW_RATE_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_SLEW_RATE_LIMIT);

    private boolean lastOmegaZero = true;
    private Rotation2d savedAngle;

    private final PIDFController adjustController = new PIDFController(5, 0, 0, 0);

    public JoystickDrive(Joystick leftJoystick, Joystick rightJoystick) {
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

            boolean omegaZero = Utils.epsilonEquals(omega, 0);

            if (!lastOmegaZero && omegaZero) {
                savedAngle = gyroscope.getYaw();
            }

            DriveSignal signal = new DriveSignal(
                    vx * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    vy * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    omega * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    new Translation2d(),
                    !leftJoystick.getTop());
            if (omegaZero && !Utils.epsilonEquals(magnitude, 0) && savedAngle != null) {
//                signal.omega = adjustController.calculate(gyroscope.getYaw().getRadians(), savedAngle.getRadians());
            }
            swerveDrive.drive(signal);

            lastOmegaZero = omegaZero;
        }
    }
}
