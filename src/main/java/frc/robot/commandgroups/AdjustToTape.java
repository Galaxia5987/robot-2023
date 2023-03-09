package frc.robot.commandgroups;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.utils.controllers.PIDFController;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class AdjustToTape extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final PIDFController yController;
    private final ProfiledPIDController rotationController;
    private final TargetAdjustInputsAutoLogged inputs = new TargetAdjustInputsAutoLogged();
    private final double desiredYaw;
    private final double desiredAbsoluteYaw;
    private final DoubleSupplier xSupplier;
    private final ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public AdjustToTape(DoubleSupplier xSupplier, double desiredYaw, double desiredAbsoluteYaw) {
        this.xSupplier = xSupplier;
        this.desiredYaw = desiredYaw;
        this.desiredAbsoluteYaw = desiredAbsoluteYaw;
        yController = new PIDFController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
        rotationController = new ProfiledPIDController(
                SwerveConstants.TARGET_ROTATION_Kp,
                SwerveConstants.TARGET_ROTATION_Ki,
                SwerveConstants.TARGET_ROTATION_Kd, new TrapezoidProfile.Constraints(10, 5));
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        yController.setTolerance(1.0);
        rotationController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        Rotation2d robotAngle = gyroscope.getYaw();
        ChassisSpeeds speeds = new ChassisSpeeds();
        var absoluteYaw = limelight.getAbsoluteYaw(robotAngle);
        var yaw = limelight.getYaw();
        inputs.robotAngle = robotAngle.getDegrees();

        if (yaw.isPresent() && absoluteYaw.isPresent()) {
            inputs.yaw = yaw.get().getDegrees();
            speeds.vyMetersPerSecond = yController.calculate(absoluteYaw.get().getSin(), Math.sin(desiredAbsoluteYaw));
            speeds.omegaRadiansPerSecond = rotationController.calculate(yaw.get().getRadians(), desiredYaw);
            inputs.outputVy = speeds.vyMetersPerSecond;
            inputs.outputOmega = speeds.omegaRadiansPerSecond;
        }

        speeds.vxMetersPerSecond = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1)
                * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND;

        swerveDrive.drive(new DriveSignal(speeds, new Translation2d(), true));

        Logger.getInstance().processInputs("AdjustToTarget", inputs);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
