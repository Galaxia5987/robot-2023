package frc.robot.utils.valuetuner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.util.HashSet;
import java.util.Set;

/**
 * The class is used to update the value of the web constant through the networktables.
 */
public class NetworkTableConstant implements WebConstant {
    private static final Set<NetworkTableConstant> constants = new HashSet<>();
    private static boolean initializedConstants = false;

    private final String table;
    private final String key;
    private double defaultValue;

    NetworkTableConstant(String table, String key, double defaultValue) {
        this.table = table;
        this.key = key;
        this.defaultValue = defaultValue;
        if (!initializedConstants) {
            constants.add(this); // lazy initialization.
        } else {
            initialize();
        }
    }

    /**
     * Initializes all the constants.
     * Should be used only in the {@link Robot#robotInit()} method.
     */
    public static void initializeAllConstants() {
        if (!initializedConstants) {
//            BASE_TABLE = NetworkTableInstance.getDefault().getTable("value-tuner");
            constants.forEach(NetworkTableConstant::initialize);
            constants.clear();
            initializedConstants = true;
        }
    }

    /**
     * Initialize the constant.
     */
    private void initialize() {
        set(defaultValue);
    }

    /**
     * Gets the value of the constant.
     *
     * @return the value of the constant or the default value.
     */
    @Override
    public double get() {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    @Override
    public void set(double value) {
        defaultValue = value;
        SmartDashboard.setDefaultNumber(key, value);
    }
}
