package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.LoggedSubsystem;

import java.util.Optional;
import java.util.OptionalDouble;

public class Limelight extends LoggedSubsystem<LimelightLogInputs> {
    public static Limelight INSTANCE = null;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry tv = table.getEntry("tv");
    private final NetworkTableEntry ts = table.getEntry("ts");
    private final NetworkTableEntry tid = table.getEntry("tid");
    private LimelightLogInputs inputs = LimelightLogInputs.getInstance();

    private Limelight() {
        super(new LimelightLogInputs());
        inputs = LimelightLogInputs.getInstance();
        PortForwarder.add(5801, "limelight.local", 5801);
    }

    public Limelight getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new Limelight();
        }
        return INSTANCE;
    }

    public boolean hasTargets() {
        return tv.getDouble(0) != 0;
    }

    public int getTagId() {
        return tid.getHandle();
    }

    public Rotation2d getPitch() {
        return new Rotation2d(ty.getDouble(0));
    }

    public OptionalDouble getTargetDistance() {
        double totalPitch = VisionConstants.CAMERA_PITCH + getPitch().getRadians();
        if (hasTargets()) {
            return OptionalDouble.of(tx.getDouble(0) * Math.sin(totalPitch));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getYaw() {
        if (hasTargets()) {
            return OptionalDouble.of(ts.getDouble(0));
        }
        return OptionalDouble.empty();
    }

    public Optional<Pose2d> estimatePose(Rotation2d robotAngle) {
        double absouluteAngle = robotAngle.getRadians() + getYaw().getAsDouble();
        double xTargetDistance = getTargetDistance().getAsDouble() * Math.sin(absouluteAngle);
        double yTargetDistance = getTargetDistance().getAsDouble() * Math.cos(absouluteAngle);
        if (hasTargets()) {
            double xDistance = xTargetDistance-VisionConstants.UPPER_CONE_TARGET11_X_DISTANCE;
            double yDistance = yTargetDistance-VisionConstants.UPPER_CONE_TARGET11_Y_DISTANCE;
            Translation2d translation2d = new Translation2d(xDistance, yDistance);
            return Optional.of(new Pose2d(translation2d, robotAngle));
        }
        return Optional.empty();
    }

    @Override
    public String getSubsystemName() {
        return "Limelight";
    }

    @Override
    public void updateInputs() {
        inputs.hasTargets = hasTargets();
        getYaw().ifPresent((value) -> inputs.yaw = value);
        inputs.tagId = getTagId();
        getTargetDistance().ifPresent((value) -> inputs.targetDistance = value);

    }
}
