package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.units.UnitModel;

public class ArmAccelerationCalculation {
    private final Arm arm = Arm.getInstance();
    private final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);
    private final Timer timer = new Timer();

    private double prevShoulderVelocity;
    private double prevElbowVelocity;
    private double time;
    private double time2 = 0;
    private double shoulderVelocity;
    private double elbowVelocity;
    private double shoulderAcceleration;
    private double elbowAcceleration;
    private double[] accelerations = new double[2];

    public double[] getArmAcceleration() {
        timer.reset();
        timer.start();
        shoulderVelocity = arm.getShoulderMotorVelocity();
        elbowVelocity = arm.getElbowMotorVelocity();
        time = timer.get();
        shoulderAcceleration = (shoulderVelocity - prevShoulderVelocity) / Math.abs(time - time2);
        elbowAcceleration = (elbowVelocity - prevElbowVelocity) / Math.abs(time - time2);
        prevShoulderVelocity = shoulderVelocity;
        prevElbowVelocity = elbowVelocity;
        time2 = timer.get();

        accelerations[0] = shoulderAcceleration;
        accelerations[2] = elbowAcceleration;
        return accelerations;
    }
}
