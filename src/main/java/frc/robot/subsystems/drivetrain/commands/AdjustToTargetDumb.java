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
import frc.robot.utils.controllers.DieterController;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class AdjustToTargetDumb extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final DieterController xController = new DieterController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final DieterController yController = new DieterController(SwerveConstants.TARGET_XY_Kp, SwerveConstants.TARGET_XY_Ki, SwerveConstants.TARGET_XY_Kd, SwerveConstants.TARGET_XY_Kf);
    private final DieterController rotationController = new DieterController(SwerveConstants.TARGET_ROTATION_Kp, SwerveConstants.TARGET_ROTATION_Ki, SwerveConstants.TARGET_ROTATION_Kd, SwerveConstants.TARGET_ROTATION_Kf);
    private Optional<Pose2d> setpointPose;
    private Position position;

    public AdjustToTargetDumb(Position position) {
        this.position = position;
        addRequirements(swerveDrive);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        var botPose = limelight.getBotPose();
        Pose2d startPose = swerveDrive.getPose();
        int id = (int) limelight.getTagId();
        boolean isFeeder = id == 4 || id == 5;
        var setpointRotation = isFeeder ? Rotation2d.fromDegrees(180) : new Rotation2d();
        if (isFeeder) {
            position = position.getFeeder();
        }
        setpointPose = botPose.map(pose2d -> startPose.plus(new Transform2d(pose2d.getTranslation()
                .minus(position.offset), new Rotation2d())));

        if (setpointPose.isPresent()) {
            gyroscope.resetYaw(botPose.get().getRotation()
                    .minus(setpointRotation));
        }

        Logger.getInstance().recordOutput("setpointPose", setpointPose.toString());
        Logger.getInstance().recordOutput("startPose", startPose.toString());
    }

    @Override
    public void execute() {
        if (setpointPose.isPresent()) {
            var pose = swerveDrive.getPose();
            var speeds = new ChassisSpeeds(
                    xController.calculate(pose.getX(), setpointPose.get().getX()),
                    yController.calculate(pose.getY(), setpointPose.get().getY()),
                    rotationController.calculate(gyroscope.getYaw().getRadians(), setpointPose.get().getRotation().getRadians())
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
        return setpointPose.isEmpty();
    }

    public enum Position {
        RIGHT(new Translation2d(-0.75, 0.69)),
        LEFT(new Translation2d(-0.75, -0.48)),
        MIDDLE(new Translation2d(-0.75, 0.13)),
        FEEDER_RIGHT(new Translation2d(-0.55, -0.48)),
        FEEDER_LEFT(new Translation2d(-0.55, 0.69));

        public final Translation2d offset;

        Position(Translation2d offset) {
            this.offset = offset;
        }

        public Position getFeeder() {
            if (this == LEFT) {
                return FEEDER_LEFT;
            } else if (this == RIGHT) {
                return FEEDER_RIGHT;
            } else {
                return this;
            }
        }
    }
}
