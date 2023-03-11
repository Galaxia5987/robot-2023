package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.utils.controllers.DieterController;

public class TurnDrivetrain extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final DieterController controller = new DieterController(
            SwerveConstants.TARGET_ROTATION_Kp,
            SwerveConstants.TARGET_ROTATION_Ki,
            SwerveConstants.TARGET_ROTATION_Kd,
            SwerveConstants.TARGET_ROTATION_Kf
    );
    private final Joystick joystick;

    public TurnDrivetrain(Joystick joystick) {
        this.joystick = joystick;
        controller.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double vx = -joystick.getY();
        double vy = -joystick.getX();

        double magnitude = Math.hypot(vx, vy);
        double angle = Math.atan2(vy, vx);
        magnitude = MathUtil.applyDeadband(magnitude, 0.05);
        vx = Math.cos(angle) * magnitude;
        vy = Math.sin(angle) * magnitude;

        vx = Math.copySign(vx * vx, vx);
        vy = Math.copySign(vy * vy, vy);

        DriveSignal signal = new DriveSignal(
                vx * SwerveConstants.MAX_VELOCITY_AUTO,
                vy * SwerveConstants.MAX_VELOCITY_AUTO,
                controller.calculate(gyroscope.getYaw().getRadians(), Math.PI),
                new Translation2d(),
                true);
        swerveDrive.drive(signal);
    }
}
