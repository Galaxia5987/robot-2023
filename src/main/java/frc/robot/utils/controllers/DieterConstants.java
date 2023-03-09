package frc.robot.utils.controllers;

import com.pathplanner.lib.auto.PIDConstants;

public class DieterConstants extends PIDConstants {
    public double kDieter;

    public DieterConstants(double kP, double kI, double kD, double kDieter, double period) {
        super(kP, kI, kD, period);
        this.kDieter = kDieter;
    }

    public DieterConstants(double kP, double kI, double kD, double kDieter) {
        super(kP, kI, kD);
        this.kDieter = kDieter;
    }
}
