package frc.robot.subsystems.LimeLightModule;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.LoggedSubsystem;


public class LimeLight extends LoggedSubsystem<LimelightLogInputs> {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry pipeline = table.getEntry("pipeline");


    public LimeLight() {
        super(new LimelightLogInputs());
        loggerInputs.x = 0;
        loggerInputs.y = 0;
        loggerInputs.a = 0;
        loggerInputs.v = false;
        loggerInputs.pipeLine = 0;
    }

    public boolean hasTarget() {
        return loggerInputs.v;
    }

    public double getHorizantalOffset() {
        return loggerInputs.x;
    }

    public double getVerticalOffset() {
        return loggerInputs.y;
    }

    public double getTargetPrecentage() {
        return loggerInputs.a;
    }

    public double getPipeLine() {
        return loggerInputs.pipeLine;
    }

    @Override
    public void updateInputs() {
        loggerInputs.x = tx.getDouble(0);
        loggerInputs.y = ty.getDouble(0);
        loggerInputs.a = ta.getDouble(0);
        loggerInputs.v = tv.getBoolean(false);
        loggerInputs.pipeLine = pipeline.getDouble(0);
    }

    @Override
    public String getSubsystemName() {
        return null;
    }
}
