package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
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

    private Limelight() {
        super(new LimelightLogInputs());
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5802, "limelight.local", 5802);
        PortForwarder.add(5803, "limelight.local", 5803);
        PortForwarder.add(5804, "limelight.local", 5804);
        PortForwarder.add(5805, "limelight.local", 5805);
    }

    /**
     * if there is no instance of the limelight class it creates one and returns it
     * @return
     */
    public static Limelight getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Limelight();
        }
        return INSTANCE;
    }

    /**
     * checks weather the limelight can see any targets
     * @return
     */
    public boolean hasTargets() {
        return tv.get() == 1;
    }

    /**
     * returns the id of the aprilTag the limelight sees
     * @return
     */
    public long getTagId() {
        return tid.get();
    }


    /**
     * returns the pitch from the robot to the target
     * @return
     */
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(ty.get());
    }

    /**
     *returns the distance from the target
     * @return
     */
    public OptionalDouble getTargetDistance() {
        double totalPitch = Constants.CAMERA_PITCH + getPitch().getRadians();
        double error = Constants.CAMERA_HEIGHT-Constants.TARGET_HEIGHT;
        if (hasTargets()) {
            return OptionalDouble.of(Math.abs(error / Math.tan(totalPitch)));
        }
        return OptionalDouble.empty();
    }

    /**
     * returns the yaw
     * @return
     */
    public OptionalDouble getYaw() {
        if (hasTargets()) {
            return OptionalDouble.of(ts.get());
        }
        return OptionalDouble.empty();
    }

    /**
     * estimates the position of the robot
     * @param robotAngle
     * @return
     */
    public Optional<Pose2d> estimatePose(Rotation2d robotAngle) {
        double absouluteAngle = robotAngle.getRadians() + getYaw().getAsDouble();
        double xTargetDistance = getTargetDistance().getAsDouble() * Math.sin(absouluteAngle);
        double yTargetDistance = getTargetDistance().getAsDouble() * Math.cos(absouluteAngle);
        if (hasTargets()) {
            double xDistance = xTargetDistance - Constants.UPPER_CONE_TARGET11_X_DISTANCE;
            double yDistance = yTargetDistance - Constants.UPPER_CONE_TARGET11_Y_DISTANCE;
            Translation2d translation2d = new Translation2d(xDistance, yDistance);
            return Optional.of(new Pose2d(translation2d, robotAngle));
        }
        return Optional.empty();
    }

    @Override
    public String getSubsystemName() {
        return "Vision";
    }

    /**
     * updates the limelight log inputs
     */
    @Override
    public void updateInputs() {
        loggerInputs.hasTargets = hasTargets();
        getYaw().ifPresent((value) -> loggerInputs.yaw = value);
        loggerInputs.tagId = getTagId();
        getTargetDistance().ifPresent((value) -> loggerInputs.targetDistance = value);

    }
}
