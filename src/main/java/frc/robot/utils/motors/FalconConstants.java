package frc.robot.utils.motors;

public class FalconConstants {
    public static final double NOMINAL_VOLTAGE = 12.0; // [V]
    public static final double STALL_CURRENT = 0; // [A]
    public static final double STALL_TORQUE = 0; // [N*m]
    public static final double FREE_SPEED = (6328 / 60.0) / (2 * Math.PI); // [rad/s]
    public static final double Kt = STALL_TORQUE / STALL_CURRENT; // [N*m/A]
    public static final double Kv = FREE_SPEED / NOMINAL_VOLTAGE; // [rad/s/V]
    public static final double R = NOMINAL_VOLTAGE / STALL_CURRENT; // [Ohm]
}
