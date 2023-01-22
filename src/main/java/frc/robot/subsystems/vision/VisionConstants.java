package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionConstants {
    public static final double CAMERA_HEIGHT = 1.1; //[m]
    public static final double CAMERA_PITCH = -0.611; //[radian]
    public static final double UPPER_CONE_TARGET_TAPE_HEIGHT = 1.17; //[m]
    public static final double LOWER_CONE_TARGET_TAPE_HEIGHT = 0.65; //[m]
    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.02;
    public static final Translation2d CENTER_POSE = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
    public static final Pose3d UPPER_CONE_LEFT_GRID_TARGET = new Pose3d(16.178, 4.99, 1.17, new Rotation3d());
    public static final Translation2d CUBE_ID1_POSE = CENTER_POSE.plus(new Translation2d(7.24310, -2.93659)).minus(new Translation2d(1.0, 0));

}
