package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;

public class AdjustToTape extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final PIDController yController;
    private final ProfiledPIDController rotationController;
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public AdjustToTape() {
        yController = new PIDController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd);
        rotationController = new ProfiledPIDController(
                SwerveConstants.TARGET_ROTATION_Kp,
                SwerveConstants.TARGET_ROTATION_Ki,
                SwerveConstants.TARGET_ROTATION_Kd, new TrapezoidProfile.Constraints(10, 5));
    }

    @Override
    public void initialize() {
        yController.setTolerance(0.01);
        rotationController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        Rotation2d robotAngle = gyroscope.getYaw().plus(Rotation2d.fromDegrees(180));
        var yaw = limelight.getYaw();
        ChassisSpeeds speeds = new ChassisSpeeds();

        if (yaw.isPresent()) {
            Rotation2d absoluteYaw = yaw.get().plus(robotAngle);
            speeds.vyMetersPerSecond = yController.calculate(absoluteYaw.getSin(), 0);
            speeds.omegaRadiansPerSecond = rotationController.calculate(yaw.get().getRadians(), 0);
        } else {
            speeds = lastSpeeds;
        }

        swerveDrive.drive(new DriveSignal(speeds, new Translation2d(), true));

        lastSpeeds = speeds;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return yController.atSetpoint() && rotationController.atSetpoint();
    }
}
