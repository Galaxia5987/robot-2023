package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class XboxDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final XboxController xboxController;

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

        double magnitude = Math.hypot(vx, vy);
        double angle = Math.atan2(vy, vx);
        magnitude = MathUtil.applyDeadband(magnitude, 0.2);
        magnitude = Math.copySign(magnitude * magnitude, magnitude);

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
