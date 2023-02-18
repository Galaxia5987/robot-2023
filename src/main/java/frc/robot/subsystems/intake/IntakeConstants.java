package frc.robot.subsystems.intake;

import frc.robot.utils.units.Units;

public class IntakeConstants {
    public static final double GEAR_RATIO = 35.26;
    public static final double kP = 4;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double FALCON_TICKS_PER_ROTATION = 2048;
    public static final double TICKS_PER_DEGREE = FALCON_TICKS_PER_ROTATION / 360 * GEAR_RATIO;

    public static final double ANGLE_UP = 90; //[deg]
    public static final double ANGLE_DOWN = -29; //[deg]

    public static final double INTAKE_POWER = 0.6; //[%]
    public static final double INTAKE_ANGLE_VELOCITY = Units.rpmToRps(1100) * FALCON_TICKS_PER_ROTATION * 10; //[RPM]
    public static final double INTAKE_ANGLE_MAX_ACCELERATION = INTAKE_ANGLE_VELOCITY / 0.5; //[RPM/s]
    public static final double MAX_CURRENT = 50;

    public static final double ANGLE_MOTOR_POWER = 0.2;
}
