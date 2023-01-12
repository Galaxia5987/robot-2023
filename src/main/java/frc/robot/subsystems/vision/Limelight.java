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
    private final BooleanSubscriber tv = table.getBooleanTopic("tv").subscribe(false);
    private final DoubleSubscriber ts = table.getDoubleTopic("ts").subscribe(0.0);
    private final IntegerSubscriber tid = table.getIntegerTopic("tid").subscribe(0);

    private Limelight() {
        super(new LimelightLogInputs());
        PortForwarder.add(5801, "limelight.local", 5801);
    }

    public Limelight getInstance() {


        if (INSTANCE == null) {
            INSTANCE = new Limelight();
        }
        return INSTANCE;
    }

    public boolean hasTargets() {
        return tv.get();
    }

    public long getTagId() {
        return tid.get();
    }

    public Rotation2d getPitch() {
        return new Rotation2d(ty.get());
    }

    public OptionalDouble getTargetDistance() {
        double totalPitch = Constants.CAMERA_PITCH + getPitch().getRadians();
        if (hasTargets()) {
            return OptionalDouble.of(tx.get() * Math.sin(totalPitch));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getYaw() {
        if (hasTargets()) {
            return OptionalDouble.of(ts.get());
        }
        return OptionalDouble.empty();
    }

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

    public void periodic(){
        updateInputs();
    }

    @Override
    public String getSubsystemName() {
        return "Limelight";
    }

    @Override
    public void updateInputs() {
        loggerInputs.hasTargets = hasTargets();
        getYaw().ifPresent((value) -> loggerInputs.yaw = value);
        loggerInputs.tagId = getTagId();
        getTargetDistance().ifPresent((value) -> loggerInputs.targetDistance = value);

    }
}
