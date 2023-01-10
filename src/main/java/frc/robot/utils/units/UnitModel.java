package frc.robot.utils.units;

public class UnitModel {
    private final double ticksPerUnit;

    public UnitModel(double ticksPerUnit) {
        this.ticksPerUnit = ticksPerUnit;
    }

    /**
     * converts from ticks to units
     *
     * @param ticks the given ticks
     * @return the corresponding value of units
     */
    public double toUnits(double ticks) {
        return ticks / ticksPerUnit;
    }

    /**
     * converts from units to ticks
     *
     * @param units the given units
     * @return the corresponding number of ticks
     */
    public int toTicks(double units) {
        return (int) (ticksPerUnit * units);
    }

    /**
     * converts from ticks per 100ms to velocity [m/s]
     *
     * @param ticks100ms the number of ticks per 100ms
     * @return the corresponding velocity value [m/s]
     */
    public double toVelocity(double ticks100ms) {
        return (ticks100ms / ticksPerUnit) * 10;
    }

    /**
     * converts from velocity to ticks per 100ms
     *
     * @param velocity the velocity in [m/s]
     * @return the number of ticks per 100ms
     */
    public int toTicks100ms(double velocity) {
        return (int) (velocity * ticksPerUnit / 10);
    }

    @Override
    public String toString() {
        return "TicksPerUnit:" + ticksPerUnit;
    }
}
