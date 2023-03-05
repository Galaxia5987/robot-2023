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

public class AdjustToTargetDumb extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();

    private Optional<Pose2d> finalPose;
    private final Position position;

    private final PIDFController xController = new PIDFController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final PIDFController yController = new PIDFController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final PIDFController rotationController = new PIDFController(SwerveConstants.TARGET_ROTATION_Kp, SwerveConstants.TARGET_ROTATION_Ki, SwerveConstants.TARGET_ROTATION_Kd, SwerveConstants.TARGET_ROTATION_Kf);

    public AdjustToTargetDumb(Position position) {
        this.position = position;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        var botPose = limelight.getBotPose();
        Pose2d startPose = swerveDrive.getPose();

        finalPose = botPose.map(pose2d -> startPose.plus(new Transform2d(pose2d.getTranslation()
                .minus(position.offset), new Rotation2d())));

        if (finalPose.isPresent()) {
            gyroscope.resetYaw(botPose.get().getRotation());
        }

        Logger.getInstance().recordOutput("finalPose", finalPose.toString());
        Logger.getInstance().recordOutput("startPose", startPose.toString());
    }

    @Override
    public void execute() {
        if (finalPose.isPresent()) {
            var pose = swerveDrive.getPose();
            var speeds = new ChassisSpeeds(
                    xController.calculate(pose.getX(), finalPose.get().getX()),
                    yController.calculate(pose.getY(), finalPose.get().getY()),
                    rotationController.calculate(gyroscope.getYaw().getRadians(), 0)
            );

            swerveDrive.drive(
                    new DriveSignal(
                            speeds,
                            new Translation2d(),
                            true
                    )
            );
        }
    }

    @Override
    public boolean isFinished() {
        return finalPose.isEmpty();
    }

    public enum Position {
        RIGHT(new Translation2d(-0.75, 0.69)),
        LEFT(new Translation2d(-0.75, -0.48)),
        MIDDLE(new Translation2d(-0.75, 0.13));

        public final Translation2d offset;

        Position(Translation2d offset) {
            this.offset = offset;
        }

        public Pose2d getFinalPose(Pose2d currentPose) {
            return new Pose2d(currentPose.getTranslation().minus(this.offset), new Rotation2d());
        }
    }
}
