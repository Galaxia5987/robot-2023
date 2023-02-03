package frc.robot;

public final class Ports {

    public static class ArmPorts {
        public static final int SHOULDER_MAIN_MOTOR = 0;
        public static final int SHOULDER_AUX_MOTOR = 0;
        public static final int ELBOW_MAIN_MOTOR = 0;
        public static final int ELBOW_AUX_MOTOR = 0;
        public static final int SHOULDER_ENCODER = 0;
        public static final int ELBOW_ENCODER = 0;
    }

    public static final class SwerveDrive {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 7;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 4;
        public static final int REAR_LEFT_MODULE_DRIVE_MOTOR_ID = 1;
        public static final int REAR_LEFT_MODULE_STEER_MOTOR_ID = 6;
        public static final int REAR_RIGHT_MODULE_DRIVE_MOTOR_ID = 5;
        public static final int REAR_RIGHT_MODULE_STEER_MOTOR_ID = 2;

        public static boolean FRONT_LEFT_DRIVE_INVERTED = true;
        public static boolean FRONT_LEFT_ANGLE_INVERTED = true;
        public static boolean FRONT_RIGHT_DRIVE_INVERTED = true;
        public static boolean FRONT_RIGHT_ANGLE_INVERTED = true;
        public static boolean REAR_LEFT_DRIVE_INVERTED = true;
        public static boolean REAR_LEFT_ANGLE_INVERTED = true;
        public static boolean REAR_RIGHT_DRIVE_INVERTED = true;
        public static boolean REAR_RIGHT_ANGLE_INVERTED = true;
    }

    public static class Leds {
        public static final int LED = 0;
    }

    public static class Gripper {
        public static final int SOLENOID = 0;

    }

    public static class Intake {
        public static final int BEAM_BREAKER_SENSOR = 0;
        public static final int INTAKE_MOTOR = 0;
        public static final int ANGLE_MOTOR = 0;
        public static final boolean POWER_INVERTED = false;
        public static final boolean ANGLE_INVERTED = false;
    }
}

