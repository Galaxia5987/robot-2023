package frc.robot.utils.math;

public class InterpolatingDoubleMap extends InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> {

    public InterpolatingDoubleMap(int maximumSize) {
        super(maximumSize);
    }

    public InterpolatingDoubleMap() {
        super();
    }

    public InterpolatingDouble put(double a, double b) {
        return super.put(new InterpolatingDouble(a), new InterpolatingDouble(b));
    }
}
