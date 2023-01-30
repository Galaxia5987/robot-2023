package frc.robot.subsystems.arm;


public class AccelerationCalculation {
    private double currentVelocity;
    private double prevVelocity;
    private double currentTime;
    private double prevTime;

    public void addVelocity(double sVelocity, double time) {
        prevVelocity = currentVelocity;
        currentVelocity = sVelocity;

        prevTime = currentTime;
        currentTime = time;
    }

    public double getAcceleration() {
        return (currentVelocity - prevVelocity) / (prevTime - currentTime);
    }
}
