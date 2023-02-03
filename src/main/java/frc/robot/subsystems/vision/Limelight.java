package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.AllianceFlipUtil;

import java.util.Optional;
import java.util.OptionalDouble;

public class Limelight extends LoggedSubsystem<LimelightLogInputs> {
    private static Limelight INSTANCE = null;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber tx = table.getDoubleTopic("tx").subscribe(0.0);
    private final DoubleSubscriber ty = table.getDoubleTopic("ty").subscribe(0.0);
    private final IntegerSubscriber tv = table.getIntegerTopic("tv").subscribe(0);
    private final DoubleSubscriber ts = table.getDoubleTopic("ts").subscribe(0.0);
    private final IntegerSubscriber tid = table.getIntegerTopic("tid").subscribe(0);
    private final IntegerSubscriber getpipe = table.getIntegerTopic("getpipe").subscribe(0); //TODO: check vision pipelines
    private final DoubleArraySubscriber botPose = table.getDoubleArrayTopic("botpose").subscribe(new double[6]);

    private final AprilTagFieldLayout aprilTagFieldLayout;

    private Limelight() {
        super(new LimelightLogInputs());
        for (int i = 5800; i <= 5805; i++) {
            PortForwarder.add(i, "limelight.local", i);
        }
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2023ChargedUp.m_resourceFile
            );
        } catch (Throwable t) {
            throw new RuntimeException(t);
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

    public Pipeline getPipeline() {
        return Pipeline.fromIndex((int) getpipe.get());
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
    public Optional<Rotation2d> getYaw() {
        if (hasTargets()) {
            return Optional.of(Rotation2d.fromDegrees(ts.get()));
        }
        return Optional.empty();
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
        double absoluteAngle = robotAngle.getRadians() + getYaw().orElse(Rotation2d.fromDegrees(0)).getRadians();
        double xTargetDistance = getTargetDistance(target.getZ()).orElse(0) * Math.sin(absoluteAngle);
        double yTargetDistance = getTargetDistance(target.getZ()).orElse(0) * Math.cos(absoluteAngle);
        double xDistance = xTargetDistance - target.getX();
        double yDistance = yTargetDistance - target.getY();
        Translation2d translation2d = new Translation2d(xDistance, yDistance);
        return Optional.of(new Pose2d(translation2d, robotAngle));
    }

    public Optional<Pose2d> getAprilTagTarget() {
        int id = (int) getTagId();
        var pose3d = aprilTagFieldLayout.getTagPose(id);
        if (pose3d.isPresent()) {
            var pose2d = pose3d.get().toPose2d();
            Translation2d withOffset;

            if (id < 5) {
                if (id == 4) {
                    withOffset = pose2d.getTranslation().minus(VisionConstants.DOUBLE_SUBSTATION_ADJUST_OFFSET);
                } else {
                    withOffset = pose2d.getTranslation().minus(VisionConstants.TARGET_ADJUST_OFFSET);
                }
            } else {
                if (id == 8) {
                    withOffset = pose2d.getTranslation().plus(VisionConstants.DOUBLE_SUBSTATION_ADJUST_OFFSET);
                } else {
                    withOffset = pose2d.getTranslation().plus(VisionConstants.TARGET_ADJUST_OFFSET);
                }
            }

            return Optional.of(new Pose2d(withOffset, pose2d.getRotation().plus(Rotation2d.fromDegrees(180))));
        }
        return Optional.empty();
    }

    public Optional<Pose2d> getAprilTagTarget(DriverStation.Alliance alliance) {
        var target = getAprilTagTarget();
        if (target.isPresent()) {
            var pose = AllianceFlipUtil.apply(alliance, target.get());
            return Optional.of(pose);
        }
        return Optional.empty();
    }

    public Optional<Pose2d> getBotPose() {
        long id = getTagId();
        if (id > 0 && id < 9) {
            return Optional.of(
                    VisionConstants.CENTER_POSE.plus(
                            new Transform2d(new Translation2d(botPose.get()[0], botPose.get()[1]), Rotation2d.fromDegrees(botPose.get()[5])))
            );
        }
        return Optional.empty();
    }

    public Optional<Pose2d> getBotPose(DriverStation.Alliance alliance) {
        var botPose = getBotPose();
        if (botPose.isPresent()) {
            var pose = AllianceFlipUtil.apply(alliance, botPose.get());
            return Optional.of(pose);
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
        getYaw().ifPresent((value) -> loggerInputs.yaw = value.getDegrees());
        loggerInputs.tagId = getTagId();
        getTargetDistance(VisionConstants.UPPER_CONE_TARGET_TAPE_HEIGHT).ifPresent((value) -> loggerInputs.targetDistance = value);
        getTargetDistance(VisionConstants.LOWER_CONE_TARGET_TAPE_HEIGHT).ifPresent((value) -> loggerInputs.targetDistance = value);
        getAprilTagTarget().ifPresent((value) -> loggerInputs.aprilTagTarget = value);
    }

    public enum Pipeline {
        APRIL_TAG_PIPELINE(0),
        REFLECTIVE_TAPE_PIPELINE(1);

        public final int index;

        Pipeline(int index) {
            this.index = index;
        }

        public static Pipeline fromIndex(int index) {
            if (index == REFLECTIVE_TAPE_PIPELINE.index) {
                return REFLECTIVE_TAPE_PIPELINE;
            }
            return APRIL_TAG_PIPELINE;
        }
    }
}
