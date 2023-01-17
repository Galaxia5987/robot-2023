package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final double CAMERA_HEIGHT = 1.1; //[m]
    public static final double CAMERA_PITCH = -0.611; //[radian]
    public static final Pose3d UPPER_CONE_LEFT_GRID_TARGET = new Pose3d(16.178, 4.99, 1.17, new Rotation3d(0, 0,0)); //[m]
}
