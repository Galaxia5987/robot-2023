package frc.robot.utils.valuetuner;

/**
 * A wrapper class to the constant, that is being used to return the value in the competition.
 */
public class WrapperConstant implements WebConstant {
    private double defaultValue;

    WrapperConstant(double defaultValue) {
        this.defaultValue = defaultValue;
    }

    /**
     * Gets the default value of the constant.
     *
     * @return the default value.
     */
    @Override
    public double get() {
        return defaultValue;
    }

    @Override
    public void set(double value) {
        defaultValue = value;
    }
}
