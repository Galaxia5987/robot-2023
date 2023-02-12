package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;

public class VisionConstants {
    public static final double CAMERA_HEIGHT = 0.76; //[m]
    public static final double CAMERA_PITCH = 0.0872665; //[radian]
    public static final double UPPER_CONE_TARGET_TAPE_HEIGHT = 1.17; //[m]
    public static final double LOWER_CONE_TARGET_TAPE_HEIGHT = 0.65; //[m]
    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.02;
    public static final Pose2d CENTER_POSE = new Pose2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, Rotation2d.fromDegrees(0));
    public static final Translation2d TARGET_ADJUST_OFFSET = new Translation2d(1.0, 0);
    public static final Translation2d DOUBLE_SUBSTATION_ADJUST_OFFSET = new Translation2d(1.0, 0);
    public static final Translation2d DOUBLE_SUBSTATION_HORIZONTAL_ADJUST_OFFSET = new Translation2d(1.0, 0);
    // Red april tag targets
//    public static final Translation2d CUBE_ID1_POSE = CENTER_POSE.plus(new Translation2d(5.8, -2.93659));
    public static final Translation2d CUBE_ID1_POSE = CENTER_POSE.getTranslation().plus(new Translation2d(7.24310, -2.93659)).minus(TARGET_ADJUST_OFFSET);
    public static final Translation2d CUBE_ID2_POSE = CENTER_POSE.getTranslation().plus(new Translation2d(7.24310, -1.26019)).minus(TARGET_ADJUST_OFFSET);
    public static final Translation2d CUBE_ID3_POSE = CENTER_POSE.getTranslation().plus(new Translation2d(7.24310, 0.41621)).minus(TARGET_ADJUST_OFFSET);
    public static final Translation2d DOUBLE_SUBSTATION_ID4_POSE = CENTER_POSE.getTranslation().plus(new Translation2d(7.90832, 2.74161)).minus(DOUBLE_SUBSTATION_ADJUST_OFFSET);
    public static final Translation2d CENTER_TRANSLATION = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
    // Blue april tag targets
    public static final Translation2d CUBE_ID5_POSE = CENTER_TRANSLATION.plus(new Translation2d(-7.24310, 2.93659)).plus(TARGET_ADJUST_OFFSET);
    public static final Translation2d CUBE_ID6_POSE = CENTER_TRANSLATION.plus(new Translation2d(-7.24310, 1.26019)).plus(TARGET_ADJUST_OFFSET);
    public static final Translation2d CUBE_ID7_POSE = CENTER_TRANSLATION.plus(new Translation2d(-7.24310, -0.41621)).plus(TARGET_ADJUST_OFFSET);
    public static final Translation2d DOUBLE_SUBSTATION_ID8_POSE = CENTER_TRANSLATION.plus(new Translation2d(-7.90832, -2.74161)).plus(DOUBLE_SUBSTATION_ADJUST_OFFSET);
    // Red april tag targets

    public static final Pose3d UPPER_CONE_LEFT_GRID_TARGET = new Pose3d(16.178, 4.99, 1.17, new Rotation3d());

    public static Translation2d getTargetDesiredTranslation(int id) {
        switch (id) {
            case 1:
                return CUBE_ID1_POSE;
            case 2:
                return CUBE_ID2_POSE;
            case 3:
                return CUBE_ID3_POSE;
            case 4:
                return DOUBLE_SUBSTATION_ID4_POSE;
            case 5:
                return CUBE_ID5_POSE;
            case 6:
                return CUBE_ID6_POSE;
            case 7:
                return CUBE_ID7_POSE;
            case 8:
                return DOUBLE_SUBSTATION_ID8_POSE;
            default:
                return null;
        }
    }
}
