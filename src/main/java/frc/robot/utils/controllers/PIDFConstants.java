package frc.robot.utils.controllers;

import com.pathplanner.lib.auto.PIDConstants;

public class PIDFConstants extends PIDConstants {
    public double kF;

    public PIDFConstants(double kP, double kI, double kD, double kF, double period) {
        super(kP, kI, kD, period);
        this.kF = kF;
    }

    public PIDFConstants(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        this.kF = kF;
    }
}
