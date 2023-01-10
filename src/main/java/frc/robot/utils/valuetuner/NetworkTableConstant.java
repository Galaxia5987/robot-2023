package frc.robot.utils.valuetuner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

import java.util.HashSet;
import java.util.Set;

/**
 * The class is used to update the value of the web constant through the networktables.
 */
public class NetworkTableConstant implements WebConstant {
    private static final Set<NetworkTableConstant> constants = new HashSet<>();
    private static NetworkTable BASE_TABLE = null;
    private static boolean initializedConstants = false;

    private final String table;
    private final String key;
    private double defaultValue;
    private NetworkTableEntry constant;

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
            BASE_TABLE = NetworkTableInstance.getDefault().getTable("value-tuner");
            constants.forEach(NetworkTableConstant::initialize);
            constants.clear();
            initializedConstants = true;
        }
    }

    /**
     * Initialize the constant.
     */
    private void initialize() {
        constant = BASE_TABLE.getSubTable(table).getEntry(key);
        constant.setDouble(defaultValue);
    }

    /**
     * Gets the value of the constant.
     *
     * @return the value of the constant or the default value.
     */
    @Override
    public double get() {
        return constant.getDouble(defaultValue);
    }

    @Override
    public void set(double value) {
        defaultValue = value;
        constant.setDouble(value);
    }
}
