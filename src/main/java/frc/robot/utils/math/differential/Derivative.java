package frc.robot.utils.math.differential;

import edu.wpi.first.wpilibj.Timer;

public class Derivative {

    private double value;
    private double lastValue;

    private double slope;

    private double lastTimestamp = 0;

    private Derivative derivative = null;

    public Derivative(double initialValue, double initialLastValue) {
        this.value = initialValue;
        this.lastValue = initialLastValue;
    }

    public Derivative differentiate() {
        if (derivative == null) {
            derivative = new Derivative(slope, slope);
        }
        return derivative;
    }

    public void update(double newValue) {
        double timestamp = Timer.getFPGATimestamp();

        lastValue = value;
        value = newValue;

        slope = (value - lastValue) / (timestamp - lastTimestamp);

        if (derivative != null) {
            derivative.update(slope);
        }

        lastTimestamp = timestamp;
    }

    public double get() {
        return slope;
    }
}
