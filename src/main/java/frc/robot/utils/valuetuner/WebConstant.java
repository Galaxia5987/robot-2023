package frc.robot.utils.valuetuner;

import frc.robot.Robot;

/**
 * The interface is used to update constants through the network table.
 */
public interface WebConstant {
    /**
     * Retrieve a new {@code WebConstant}.
     *
     * @param subsystem    the name of the subsystem.
     * @param key          the name of the key.
     * @param defaultValue the default value of the constant.
     * @return a new constant.
     */
    static WebConstant of(String subsystem, String key, double defaultValue) {
        if (Robot.debug) {
            return new NetworkTableConstant(subsystem, key, defaultValue);
        }
        return new WrapperConstant(defaultValue);
    }

    /**
     * Gets the current value of the constant.
     *
     * @return the value of the constant.
     */
    double get();

    void set(double value);
}
