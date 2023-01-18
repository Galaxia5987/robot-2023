package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.controllers.PIDFController;
import frc.robot.utils.units.UnitModel;

import javax.swing.*;

public class PrototypeArm extends LoggedSubsystem<PrototypeArmLogInputs> {
    public static PrototypeArm INSTANCE = null;
    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);

    public final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

    public final TalonSRX shoulderMotor = new TalonSRX(Ports.prototypeArmPorts.SHOULDER_MOTOR);
    public final TalonSRX elbowMotor = new TalonSRX(Ports.prototypeArmPorts.ELBOW_MOTOR);
    public final CANCoder shoulderEncoder = new CANCoder(Ports.prototypeArmPorts.SHOULDER_ENCODER);
    public final CANCoder elbowEncoder = new CANCoder(Ports.prototypeArmPorts.ELBOW_ENCODER);
    public final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);

    private double prevShoulderVelocity;
    private double shoulderTime2;
    private double shoulderFeedforward;
    private double prevElbowVelocity;
    private double elbowTime2;
    private double elbowFeedForward;

    public PrototypeArm() {
        super(new PrototypeArmLogInputs());
        timer.reset();
        timer.start();

        shoulderMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        shoulderMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPANSATION);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMotor.setInverted(ArmConstants.clockWise);
        shoulderMotor.config_kP(0, ArmConstants.kP);
        shoulderMotor.config_kI(0, ArmConstants.kI);
        shoulderMotor.config_kD(0, ArmConstants.kD);
        shoulderMotor.config_kF(0, shoulderFeedforward);

        elbowMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        elbowMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPANSATION);
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.setInverted(ArmConstants.clockWise);
        elbowMotor.config_kP(0, ArmConstants.kP);
        elbowMotor.config_kI(0, ArmConstants.kI);
        elbowMotor.config_kD(0, ArmConstants.kD);
        elbowMotor.config_kF(0, elbowFeedForward);
    }

    public void setShoulderJointPower(double power) {
        shoulderMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void setElbowJointPower(double power) {
        elbowMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void setShoulderJointVelocity(double velocity){
        shoulderMotor.set(ControlMode.Velocity, velocity);
    }

    public void setElbowJointVelocity(double velocity){
        elbowMotor.set(ControlMode.Velocity, velocity);
    }

    public double getShoulderJointPosition() {
        return Math.toDegrees(unitModel.toUnits(shoulderEncoder.getPosition()));
    }

    public void setShoulderJointPosition(double angle) {
        shoulderEncoder.setPosition(unitModel.toTicks(Math.toRadians(angle)));
    }

    public double getElbowJointPosition() {
        return Math.toDegrees(unitModel.toUnits(elbowEncoder.getPosition()));
    }

    public void setElbowJointPosition(double angle) {
        elbowEncoder.setPosition(unitModel.toTicks(Math.toRadians(angle)));
    }


    public double getShoulderMotorPower() {
        return shoulderMotor.getMotorOutputPercent();
    }

    public double getElbowMotorPower() {
        return elbowMotor.getMotorOutputPercent();
    }

    public double getShoulderMotorVelocity(){
        return shoulderMotor.getSelectedSensorVelocity();
    }

    public double getElbowMotorVelocity(){
        return elbowMotor.getSelectedSensorVelocity();
    }

    public void setPosition(Translation2d armLocation){
        setShoulderJointPosition(kinematics.inverseKinematics(armLocation.getX(), armLocation.getY()).shoulderAngle);
        setElbowJointPosition(kinematics.inverseKinematics(armLocation.getX(), armLocation.getY()).elbowAngle);
    }

    public String getSubsystemName() {
        return "PrototypeArm";
    }

    public void periodic(){
        timer.reset();
        timer.start();
        double shoulderVelocity = getShoulderMotorVelocity();
        double shoulderTime = timer.get();
        double shoulderAcceleration = (shoulderVelocity - prevShoulderVelocity)/ Math.abs(shoulderTime-shoulderTime2);
        prevShoulderVelocity = shoulderVelocity;
        shoulderTime2 = timer.get();

        double elbowVelocity = getElbowMotorVelocity();
        double elbowTime = timer.get();
        double elbowAcceleration = (elbowVelocity - prevElbowVelocity)/Math.abs(elbowTime-elbowTime2);
        prevElbowVelocity = elbowVelocity;
        elbowTime2 = timer.get();

        shoulderFeedforward = systemModel.calculateFeedForward(Math.toDegrees(unitModel.toUnits(getShoulderJointPosition())), Math.toDegrees(unitModel.toUnits(getElbowJointPosition())), getShoulderMotorVelocity(), getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration).shoulderFeedForward;
        elbowFeedForward = systemModel.calculateFeedForward(Math.toDegrees(unitModel.toUnits(getShoulderJointPosition())), Math.toDegrees(unitModel.toUnits(getElbowJointPosition())), getShoulderMotorVelocity(), getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration).elbowFeedForward;
    }

    public void updateInputs() {
        loggerInputs.elbowAngle = getElbowJointPosition();
        loggerInputs.shoulderAngle = getShoulderJointPosition();
        loggerInputs.elbowMotorPower = getElbowMotorPower();
        loggerInputs.shoulderMotorPower = getShoulderMotorPower();
    }
}
