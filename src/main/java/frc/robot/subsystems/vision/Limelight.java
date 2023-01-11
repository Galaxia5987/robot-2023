package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.LoggedSubsystem;

public class Limelight extends LoggedSubsystem<LimelightLogInputs> {
    public static Limelight INSTANCE = null;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry tv = table.getEntry("tv");
    private final NetworkTableEntry ts = table.getEntry("ts");
    private final NetworkTableEntry tid = table.getEntry("tid");
    private LimelightLogInputs inputs = LimelightLogInputs.getInstance();

    public Limelight getINSTANCE(){
        if (INSTANCE==null){
            INSTANCE = new Limelight();
        }
        return INSTANCE;
    }

    private Limelight() {
        super(new LimelightLogInputs());
        inputs = LimelightLogInputs.getInstance();
        PortForwarder.add(5801, "limelight.local", 5801);
    }

    public boolean hasTargets(){
        return tv.getDouble(0)!=0;
    }

    public int getTagId(){
        return tid.getHandle();
    }

    public Rotation2d getPitch(){
        return new Rotation2d(ty.getDouble(0));
    }

    public double getTargetDistance(){
        double totalPitch = VisionConstants.CAMERA_PITCH+getPitch().getRadians();
        return tx.getDouble(0)*Math.sin(totalPitch);
    }

    public Rotation2d getYaw(){
        return new Rotation2d(ts.getDouble(0));
    }

    @Override
    public String getSubsystemName() {
        return "Limelight";
    }

    @Override
    public void updateInputs() {
        inputs.hasTargets = hasTargets();
        inputs.yaw = getYaw().getRadians();
        inputs.tagId = getTagId();
        inputs.targetDistance = getTargetDistance();

    }
}
