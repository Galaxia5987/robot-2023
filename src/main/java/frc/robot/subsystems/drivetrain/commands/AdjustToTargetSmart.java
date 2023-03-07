package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.controllers.PIDFController;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class AdjustToTargetSmart extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final int id;
    private Optional<Pose2d> setPointPose = Optional.empty();


    private final PIDFController xController = new PIDFController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final PIDFController yController = new PIDFController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final PIDFController rotationController = new PIDFController(SwerveConstants.TARGET_ROTATION_Kp, SwerveConstants.TARGET_ROTATION_Ki, SwerveConstants.TARGET_ROTATION_Kd, SwerveConstants.TARGET_ROTATION_Kf);

    public AdjustToTargetSmart(int id) {
        this.id = id;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        setPointPose = limelight.positionForId(id);
    }

    @Override
    public void execute() {
        var pose = swerveDrive.getPose();
        if (setPointPose.isPresent()){
            var speeds = new ChassisSpeeds(
                    xController.calculate(pose.getX(), setPointPose.get().getX()),
                    yController.calculate(pose.getY(), setPointPose.get().getY()),
                    rotationController.calculate(gyroscope.getYaw().getRadians(), setPointPose.get().getRotation().getRadians())
            );
            swerveDrive.drive(new DriveSignal(
                    speeds,
                    new Translation2d(setPointPose.get().getX(), setPointPose.get().getY()),
                    true
            ));
        }
    }

    @Override
    public boolean isFinished() {
        return setPointPose.isEmpty();
    }
}
