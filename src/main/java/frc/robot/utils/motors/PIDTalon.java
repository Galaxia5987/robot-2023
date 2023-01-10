package frc.robot.utils.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.utils.Utils;

public class PIDTalon extends TalonFX {
    public static final double EPSILON = 1e-9;

    private double lastKp;
    private double lastKi;
    private double lastKd;
    private double lastKf;
    private double lastIZone;

    private double lastMotionCruiseVelocity;
    private double lastMotionMaxAcceleration;

    public PIDTalon(int deviceNumber, String canBus) {
        super(deviceNumber, canBus);
        lastKp = 0;
        lastKi = 0;
        lastKd = 0;
        lastKf = 0;
        lastIZone = 0;
        lastMotionCruiseVelocity = 0;
        lastMotionMaxAcceleration = 0;
    }

    public PIDTalon(int deviceNumber) {
        super(deviceNumber);
        lastKp = 0;
        lastKi = 0;
        lastKd = 0;
        lastKf = 0;
        lastIZone = 0;
        lastMotionCruiseVelocity = 0;
        lastMotionMaxAcceleration = 0;
    }

    public ErrorCode updatePIDF(int slot, double kP, double kI, double kD, double kF) {
        ErrorCode[] errorCodes = new ErrorCode[]{
                ErrorCode.OK,
                ErrorCode.OK,
                ErrorCode.OK,
                ErrorCode.OK
        };

        if (updateConstant(kP, lastKp)) {
            errorCodes[0] = super.config_kP(slot, kP);
        }
        if (updateConstant(kI, lastKi)) {
            errorCodes[1] = super.config_kI(slot, kI);
        }
        if (updateConstant(kD, lastKd)) {
            errorCodes[2] = super.config_kD(slot, kD);
        }
        if (updateConstant(kF, lastKf)) {
            errorCodes[3] = super.config_kF(slot, kF);
        }
        lastKp = kP;
        lastKi = kI;
        lastKd = kD;
        lastKf = kF;

        for (ErrorCode errorCode : errorCodes) {
            if (errorCode != ErrorCode.OK) {
                return errorCode;
            }
        }

        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_IntegralZone(int slot, double iZone) {
        if (updateConstant(iZone, lastIZone)) {
            lastIZone = iZone;
            return super.config_IntegralZone(slot, iZone);
        }
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double sensorUnitsPer100ms) {
        if (updateConstant(sensorUnitsPer100ms, lastMotionCruiseVelocity)) {
            lastMotionCruiseVelocity = sensorUnitsPer100ms;
            return super.configMotionCruiseVelocity(sensorUnitsPer100ms);
        }
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionAcceleration(double sensorUnitsPer100msPerSec) {
        if (updateConstant(sensorUnitsPer100msPerSec, lastMotionMaxAcceleration)) {
            lastMotionMaxAcceleration = sensorUnitsPer100msPerSec;
            return super.configMotionAcceleration(sensorUnitsPer100msPerSec);
        }
        return ErrorCode.OK;
    }

    public boolean updateConstant(double currVal, double lastVal) {
        return Utils.deadband(currVal - lastVal, EPSILON) != 0;
    }
}
