package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class Ports {

    public static final class SwerveDrive {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 7;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 4;
        public static final int REAR_LEFT_MODULE_DRIVE_MOTOR_ID = 1;
        public static final int REAR_LEFT_MODULE_STEER_MOTOR_ID = 6;
        public static final int REAR_RIGHT_MODULE_DRIVE_MOTOR_ID = 5;
        public static final int REAR_RIGHT_MODULE_STEER_MOTOR_ID = 2;

        public static boolean FRONT_LEFT_DRIVE_INVERTED = false;
        public static boolean FRONT_LEFT_ANGLE_INVERTED = true;
        public static boolean FRONT_LEFT_ANGLE_SENSOR_PHASE = false;
        public static boolean FRONT_RIGHT_DRIVE_INVERTED = false;
        public static boolean FRONT_RIGHT_ANGLE_INVERTED = true;
        public static boolean FRONT_RIGHT_ANGLE_SENSOR_PHASE = false;
        public static boolean REAR_LEFT_DRIVE_INVERTED = false;
        public static boolean REAR_LEFT_ANGLE_INVERTED = true;
        public static boolean REAR_LEFT_ANGLE_SENSOR_PHASE = false;
        public static boolean REAR_RIGHT_DRIVE_INVERTED = false;
        public static boolean REAR_RIGHT_ANGLE_INVERTED = true;
        public static boolean REAR_RIGHT_ANGLE_SENSOR_PHASE = false;
    }

    public static class Intake {
        public static final int MOTOR = 0;
        public static final int SOLENOID = 0;
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
    }

    public static class Shooter {
        public static final int MOTOR = 1;
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
    }

    public static class Hood {
        public static final int MOTOR = 20;
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
    }

    public static class Conveyor {
        public static final int MOTOR_FROM_INTAKE = 21;
        public static final int MOTOR_TO_SHOOTER = 11;
        public static final int PRE_FLAP_BEAM = 8;
    }
}
