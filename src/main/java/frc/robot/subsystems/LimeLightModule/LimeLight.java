package frc.robot.subsystems.LimeLightModule;

import edu.wpi.first.math.geometry.Pose2d;
<<<<<<< HEAD
import edu.wpi.first.math.geometry.Rotation2d;
=======
>>>>>>> lime-light
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.LoggedSubsystem;


public class LimeLight extends LoggedSubsystem<LimelightLogInputs> {
    Constants constants = new Constants();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ts = table.getEntry("ts");
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry pipeline = table.getEntry("pipeline");


    public LimeLight() {
        super(new LimelightLogInputs());
        loggerInputs.x = 0;
        loggerInputs.y = 0;
        loggerInputs.a = 0;
        loggerInputs.s = 0;
        loggerInputs.id = 0;
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

    public double getSkewOffset(){
        return loggerInputs.s;
    }

    public double getTargetPrecentage() {
        return loggerInputs.a;
    }

public double getRobotX(){
        return ((constants.CAMERA_HEIGHT-constants.TARGET_HEIGHT)*(Math.tan(Math.toRadians(getVerticalOffset()+ constants.CAMERA_ANGLE))));
}
public double getRobotY(){
        return  (Math.tan(Math.toRadians(getHorizantalOffset()))*getRobotX());
}
<<<<<<< HEAD
   public double getRobotTheta(){
        return getHorizantalOffset()+Math.toDegrees(Math.acos(getRobotX()/getRobotY()))-90;
   }

   public Pose2d getRobotPose(){
        Translation2d robotPosition = new Translation2d(getRobotX(), getRobotY());
       Rotation2d robotRotation = new Rotation2d(getRobotTheta());
        return new Pose2d(robotPosition, robotRotation);
   }

=======

>>>>>>> lime-light
    public double getId(){
return loggerInputs.id;
    }

    public long getPipeLine() {
        return loggerInputs.pipeLine;
    }

    public void setPipeline(long pipeline) {
        loggerInputs.pipeLine = pipeline;
    }

    @Override
    public void periodic() {
        pipeline.setInteger(getPipeLine());
    }

    @Override
    public void updateInputs() {
        loggerInputs.x = tx.getDouble(0);
        loggerInputs.y = ty.getDouble(0);
        loggerInputs.a = ta.getDouble(0);
        loggerInputs.v = tv.getBoolean(false);
        loggerInputs.s = ts.getDouble(0);
        loggerInputs.id = tid.getInteger(0);
    }

    @Override
    public String getSubsystemName() {
        return null;
    }
}
