package frc.robot.commandgroups;

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
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class AdjustToTarget extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final PIDController yController;
    private final ProfiledPIDController rotationController;
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    private final TargetAdjustInputsAutoLogged inputs = new TargetAdjustInputsAutoLogged();

    public AdjustToTarget() {
        yController = new PIDController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd);
        rotationController = new ProfiledPIDController(
                SwerveConstants.TARGET_ROTATION_Kp,
                SwerveConstants.TARGET_ROTATION_Ki,
                SwerveConstants.TARGET_ROTATION_Kd, new TrapezoidProfile.Constraints(10, 5));
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        yController.setTolerance(0.01);
        rotationController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        Rotation2d robotAngle = gyroscope.getYaw().plus(Rotation2d.fromDegrees(180));
        Optional<Rotation2d> yaw = limelight.getYaw();
        ChassisSpeeds speeds = new ChassisSpeeds();
        inputs.robotAngle = robotAngle.getDegrees();

        if (yaw.isPresent()) {
            inputs.yaw = yaw.get().getDegrees();
            speeds.vyMetersPerSecond = yController.calculate(yaw.get().getCos(), 0);
            speeds.omegaRadiansPerSecond = -rotationController.calculate(yaw.get().minus(robotAngle).getRadians(), 0);
            inputs.outputVy = speeds.vyMetersPerSecond;
            inputs.outputOmega = speeds.omegaRadiansPerSecond;
        }

        swerveDrive.drive(new DriveSignal(speeds, new Translation2d(), true));

        Logger.getInstance().processInputs("AdjustToTarget", inputs);
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
