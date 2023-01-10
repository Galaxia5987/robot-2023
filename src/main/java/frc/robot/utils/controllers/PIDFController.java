package frc.robot.utils.controllers;

import edu.wpi.first.math.controller.PIDController;

public class PIDFController extends PIDController {
    private double kF;

    public PIDFController(double kp, double ki, double kd, double kF) {
        super(kp, ki, kd);
        this.kF = kF;
    }

    public PIDFController(double kp, double ki, double kd, double kF, double period) {
        super(kp, ki, kd, period);
        this.kF = kF;
    }

    public void setF(double kF) {
        this.kF = kF;
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        setPID(kP, kI, kD);
        setF(kF);
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return this.calculate(measurement);
    }

    @Override
    public double calculate(double measurement) {
        double val = super.calculate(measurement);
        val += Math.signum(val) * kF;
        return val;
    }
}
