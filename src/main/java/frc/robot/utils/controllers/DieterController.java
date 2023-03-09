package frc.robot.utils.controllers;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.Utils;

public class DieterController extends PIDController {
    private double kDieter;
    private double dieterBand = 0;

    public DieterController(double kp, double ki, double kd, double kDieter) {
        super(kp, ki, kd);
        this.kDieter = kDieter;
    }

    public DieterController(double kp, double ki, double kd, double kDieter, double period) {
        super(kp, ki, kd, period);
        this.kDieter = kDieter;
    }

    public void setDieter(double kDieter) {
        this.kDieter = kDieter;
    }

    public void setPIDF(double kP, double kI, double kD, double kDieter) {
        setPID(kP, kI, kD);
        setDieter(kDieter);
    }

    public void setDieterBand(double dieterBand) {
        this.dieterBand = dieterBand;
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return this.calculate(measurement);
    }

    @Override
    public double calculate(double measurement) {
        double val = super.calculate(measurement);
        if (!Utils.epsilonEquals(getSetpoint(), measurement, dieterBand)) {
            val += Math.copySign(kDieter, val);
        }
        return val;
    }
}
