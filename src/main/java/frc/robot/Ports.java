package frc.robot;

public final class Ports {

    public static class ArmPorts {
        public static final int SHOULDER_MAIN_MOTOR = 14;
        public static final int SHOULDER_AUX_MOTOR = 20;
        public static final int ELBOW_MAIN_MOTOR = 12;
        public static final int ELBOW_AUX_MOTOR = 10;
        public static final int SHOULDER_ENCODER = 4;
        public static final int ELBOW_ENCODER = 1;
    }

    public static final class SwerveDrive {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 6;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
        public static final int REAR_LEFT_DRIVE_MOTOR_ID = 7;
        public static final int REAR_LEFT_ANGLE_MOTOR_ID = 8;
        public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 5;
        public static final int REAR_RIGHT_ANGLE_MOTOR_ID = 2;

        public static final int[] DRIVE_IDS = {
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                REAR_LEFT_DRIVE_MOTOR_ID,
                REAR_RIGHT_DRIVE_MOTOR_ID
        };
        public static final int[] ANGLE_IDS = {
                FRONT_LEFT_ANGLE_MOTOR_ID,
                FRONT_RIGHT_ANGLE_MOTOR_ID,
                REAR_LEFT_ANGLE_MOTOR_ID,
                REAR_RIGHT_ANGLE_MOTOR_ID
        };

        public static final int FRONT_LEFT_ENCODER_ID = 5;
        public static final int FRONT_RIGHT_ENCODER_ID = 8;
        public static final int REAR_LEFT_ENCODER_ID = 2;
        public static final int REAR_RIGHT_ENCODER_ID = 7;

        public static final int[] ENCODER_IDS = {
                FRONT_LEFT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_ID,
                REAR_LEFT_ENCODER_ID,
                REAR_RIGHT_ENCODER_ID
        };

        public static boolean FRONT_LEFT_DRIVE_INVERTED = true;
        public static boolean FRONT_LEFT_ANGLE_INVERTED = true;
        public static boolean FRONT_RIGHT_DRIVE_INVERTED = true;
        public static boolean FRONT_RIGHT_ANGLE_INVERTED = true;
        public static boolean REAR_LEFT_DRIVE_INVERTED = true;
        public static boolean REAR_LEFT_ANGLE_INVERTED = true;
        public static boolean REAR_RIGHT_DRIVE_INVERTED = true;
        public static boolean REAR_RIGHT_ANGLE_INVERTED = true;

        public static final boolean[] DRIVE_INVERTED = {
                FRONT_LEFT_DRIVE_INVERTED,
                FRONT_RIGHT_DRIVE_INVERTED,
                REAR_LEFT_DRIVE_INVERTED,
                REAR_RIGHT_DRIVE_INVERTED
        };
        public static final boolean[] ANGLE_INVERTED = {
                FRONT_LEFT_ANGLE_INVERTED,
                FRONT_RIGHT_ANGLE_INVERTED,
                REAR_LEFT_ANGLE_INVERTED,
                REAR_RIGHT_ANGLE_INVERTED
        };
    }

    public static class Leds {
        public static final int LED = 0;
    }

    public static class Gripper {
        public static final int SOLENOID = 4;

    }

    public static class Intake {
        public static final int BEAM_BREAKER_SENSOR = 9;
        public static final int INTAKE_MOTOR = 17;
        public static final int ANGLE_MOTOR = 18;
        public static final boolean POWER_INVERTED = false;
        public static final boolean ANGLE_INVERTED = false;
    }

    public static class UI {
        public static final int JOYSTICK_TRIGGER = 1;
        public static final int JOYSTICK_TOP_BOTTOM_BUTTON = 2;
        public static final int JOYSTICK_TOP_LEFT_BUTTON = 3;
        public static final int JOYSTICK_TOP_RIGHT_BUTTON = 4;
    }
}

