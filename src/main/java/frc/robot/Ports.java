package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class Ports {

    public static final class SwerveDrive {
        // front right
        public static final int DRIVE_MOTOR_FR = 3;
        public static final int ANGLE_MOTOR_FR = 4;
        public static final boolean DRIVE_INVERTED_FR = false;
        public static final boolean ANGLE_INVERTED_FR = true;
        public static final boolean ANGLE_SENSOR_PHASE_FR = false;

        // front left
        public static final int DRIVE_MOTOR_FL = 7;
        public static final int ANGLE_MOTOR_FL = 8;
        public static final boolean DRIVE_INVERTED_FL = false;
        public static final boolean ANGLE_INVERTED_FL = true;
        public static final boolean ANGLE_SENSOR_PHASE_FL = false;

        // rear right
        public static final int DRIVE_MOTOR_RR = 5;
        public static final int ANGLE_MOTOR_RR = 2;
        public static final boolean DRIVE_INVERTED_RR = false;
        public static final boolean ANGLE_INVERTED_RR = true;
        public static final boolean ANGLE_SENSOR_PHASE_RR = false;

        // rear left
        public static final int DRIVE_MOTOR_RL = 1;
        public static final int ANGLE_MOTOR_RL = 6;
        public static final boolean DRIVE_INVERTED_RL = false;
        public static final boolean ANGLE_INVERTED_RL = true;
        public static final boolean ANGLE_SENSOR_PHASE_RL = false;
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
