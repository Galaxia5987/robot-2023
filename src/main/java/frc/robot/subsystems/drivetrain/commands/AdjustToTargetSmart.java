package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drivetrain.DriveSignal;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyroscope.Gyroscope;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.GridChooser;
import frc.robot.utils.controllers.DieterController;

public class AdjustToTargetSmart extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Gyroscope gyroscope = Gyroscope.getInstance();
    private final GridChooser.Position position;
    private final DieterController xController = new DieterController(SwerveConstants.TARGET_X_Kp, SwerveConstants.TARGET_X_Ki, SwerveConstants.TARGET_X_Kd, SwerveConstants.TARGET_X_Kf);
    private final DieterController yController = new DieterController(SwerveConstants.TARGET_Y_Kp, SwerveConstants.TARGET_Y_Ki, SwerveConstants.TARGET_Y_Kd, SwerveConstants.TARGET_Y_Kf);
    private final DieterController rotationController = new DieterController(SwerveConstants.TARGET_ROTATION_Kp, SwerveConstants.TARGET_ROTATION_Ki, SwerveConstants.TARGET_ROTATION_Kd, SwerveConstants.TARGET_ROTATION_Kf);
    private Pose2d setPointPose;

    public AdjustToTargetSmart(GridChooser.Position position) {
        this.position = position;
        addRequirements(swerveDrive);

        xController.setDieterBand(0.05);
        yController.setDieterBand(0.05);
        rotationController.setDieterBand(Math.toRadians(5));

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        rotationController.setTolerance(Math.toRadians(1));
    }

    @Override
    public void initialize() {
        limelight.positionForId(position.aprilTagID).ifPresentOrElse(
                pose -> setPointPose = pose,
                () -> setPointPose = null
        );
    }

    @Override
    public void execute() {
        var pose = swerveDrive.getEstimatedPose();
        if (setPointPose != null) {
            Translation2d offset;
            if (position.index % 3 == 1) {
                offset = AdjustToTargetDumb.Position.LEFT.offset;
            } else if (position.index % 3 == 2) {
                offset = AdjustToTargetDumb.Position.MIDDLE.offset;
            } else {
                offset = AdjustToTargetDumb.Position.RIGHT.offset;
            }
            var setPointPose = this.setPointPose.plus(new Transform2d(offset, new Rotation2d()));
            var speeds = new ChassisSpeeds(
                    xController.calculate(pose.getX(), setPointPose.getX()),
                    yController.calculate(pose.getY(), setPointPose.getY()),
                    rotationController.calculate(gyroscope.getYaw().getRadians(), setPointPose.getRotation().getRadians())
            );
            swerveDrive.drive(new DriveSignal(
                    speeds,
                    new Translation2d(),
                    true
            ));
        }
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() &&
                yController.atSetpoint()) ||
                setPointPose == null;
    }

    public CommandBase withFinishingCommand() {
        return this.andThen(
                new FunctionalCommand(
                        () -> setPointPose = setPointPose
                                .plus(new Transform2d(new Translation2d(-0.1, 0), new Rotation2d())),
                        () -> {
                            Transform2d error = setPointPose.minus(swerveDrive.getEstimatedPose());
                            Rotation2d heading = new Rotation2d(error.getX(), error.getY());
                            swerveDrive.drive(new DriveSignal(
                                    heading.getCos(), heading.getSin(), 0,
                                    new Translation2d(),
                                    true
                            ));
                        },
                        (b) -> {},
                        () -> swerveDrive.getEstimatedPose().minus(setPointPose)
                                .getTranslation().getNorm() < 0.01,
                        swerveDrive
                )
        );
    }
}
