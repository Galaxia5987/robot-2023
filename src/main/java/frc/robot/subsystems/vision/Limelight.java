package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.LoggedSubsystem;

import java.util.Optional;
import java.util.OptionalDouble;

public class Limelight extends LoggedSubsystem<LimelightLogInputs> {
    public static Limelight INSTANCE = null;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber tx = table.getDoubleTopic("tx").subscribe(0.0);
    private final DoubleSubscriber ty = table.getDoubleTopic("ty").subscribe(0.0);
    private final IntegerSubscriber tv = table.getIntegerTopic("tv").subscribe(0);
    private final DoubleSubscriber ts = table.getDoubleTopic("ts").subscribe(0.0);
    private final IntegerSubscriber tid = table.getIntegerTopic("tid").subscribe(0);
    private final DoubleArraySubscriber botPose = table.getDoubleArrayTopic("botpose").subscribe(new double[6]);

    private Limelight() {
        super(new LimelightLogInputs());
        for (int i = 5800; i <= 5805; i++) {
            PortForwarder.add(i, "limelight.local", i);
        }
    }

    /**
     * If there is no instance of the limelight class it creates one and returns it
     *
     * @return Limelight instance
     */
    public static Limelight getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Limelight();
        }
        return INSTANCE;
    }

    /**
     * Checks whether the limelight can see any targets
     *
     * @return whether the limelight detects any targets
     */
    public boolean hasTargets() {
        return tv.get() > 0;
    }

    /**
     * @return aprilTag id
     */
    public long getTagId() {
        return tid.get();
    }


    /**
     * @return pitch from camera to target
     */
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(ty.get());
    }

    /**
     * @return distance from target
     */
    public OptionalDouble getTargetDistance(double targetHeight) {
        if (!hasTargets()) {
            return OptionalDouble.empty();
        }
        double totalPitch = VisionConstants.CAMERA_PITCH + getPitch().getRadians();
        double error = VisionConstants.CAMERA_HEIGHT - targetHeight;
        return OptionalDouble.of(Math.abs(error * Math.tan(totalPitch)));
    }

    /**
     * @return robot yaw
     */
    public OptionalDouble getYaw() {
        if (hasTargets()) {
            return OptionalDouble.of(ts.get());
        }
        return OptionalDouble.empty();
    }

    /**
     * Estimates the position of the robot
     *
     * @param robotAngle The angle of the robot
     * @return Optional Pose2d of the robot coordinates
     */
    public Optional<Pose2d> estimatePose(Rotation2d robotAngle, Pose3d target) {
        if (!hasTargets()) {
            return Optional.empty();
        }
        double absoluteAngle = robotAngle.getRadians() + getYaw().orElse(0);
        double xTargetDistance = getTargetDistance(target.getZ()).orElse(0) * Math.sin(absoluteAngle);
        double yTargetDistance = getTargetDistance(target.getZ()).orElse(0) * Math.cos(absoluteAngle);
        double xDistance = xTargetDistance - target.getX();
        double yDistance = yTargetDistance - target.getY();
        Translation2d translation2d = new Translation2d(xDistance, yDistance);
        return Optional.of(new Pose2d(translation2d, robotAngle));
    }

    public Optional<AprilTagTarget> getAprilTagTarget() {
        int tagId = (int) getTagId();
        if (tagId > 0 && tagId < 9) {
            var currentTranslation = botPose.get();
            return Optional.of(AprilTagTarget.of(tagId,
                    VisionConstants.CENTER_POSE.plus(new Translation2d(currentTranslation[0], currentTranslation[1]))
            ));
        }
        return Optional.empty();
    }

    @Override
    public String getSubsystemName() {
        return "Limelight";
    }

    /**
     * Updates the limelight log inputs.
     */
    public void updateInputs() {
        loggerInputs.hasTargets = hasTargets();
        getYaw().ifPresent((value) -> loggerInputs.yaw = value);
        loggerInputs.tagId = getTagId();
        getTargetDistance(VisionConstants.UPPER_CONE_TARGET_TAPE_HEIGHT).ifPresent((value) -> loggerInputs.targetDistance = value);
        getTargetDistance(VisionConstants.LOWER_CONE_TARGET_TAPE_HEIGHT).ifPresent((value) -> loggerInputs.targetDistance = value);
        getAprilTagTarget().ifPresent((value) -> loggerInputs.aprilTagTarget = value);
    }

    public static class AprilTagTarget {
        public Translation2d currentTranslation;
        public Translation2d desiredTranslation;
        public Rotation2d targetHeading;
        public Rotation2d zeroHeading;
        public Rotation2d targetYaw;

        public AprilTagTarget(Translation2d currentTranslation, Translation2d desiredTranslation, Rotation2d targetHeading, Rotation2d zeroHeading, Rotation2d targetYaw) {
            this.currentTranslation = currentTranslation;
            this.desiredTranslation = desiredTranslation;
            this.targetHeading = targetHeading;
            this.zeroHeading = zeroHeading;
            this.targetYaw = targetYaw;
        }

        public static AprilTagTarget of(int id, Translation2d currentTranslation) {
            Rotation2d zeroHeading;
            Rotation2d targetHeading;
            Translation2d desiredTranslation;

            if (id < 5) {
                zeroHeading = Rotation2d.fromDegrees(180);
                targetHeading = Rotation2d.fromDegrees(0);
            } else {
                zeroHeading = Rotation2d.fromDegrees(180);
                targetHeading = Rotation2d.fromDegrees(0);
            }
            desiredTranslation = VisionConstants.getTargetDesiredTranslation(id);

            return new AprilTagTarget(currentTranslation, desiredTranslation, targetHeading, zeroHeading, targetHeading);
        }
    }
}
