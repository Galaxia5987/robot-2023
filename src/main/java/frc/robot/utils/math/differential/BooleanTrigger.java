package frc.robot.utils.math.differential;

public class BooleanTrigger {
    private boolean value;
    private boolean lastValue;

    private boolean triggered = false;
    private boolean released = false;

    public BooleanTrigger(boolean initialValue, boolean initialLastValue) {
        this.value = initialValue;
        this.lastValue = initialLastValue;
    }

    public void update(boolean newValue) {
        lastValue = value;
        value = newValue;

        triggered = value && !lastValue;
        released = !value && lastValue;
    }

    public boolean triggered() {
        return triggered;
    }

    public boolean released() {
        return released;
    }
}
