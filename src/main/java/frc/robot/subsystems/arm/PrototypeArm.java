package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class PrototypeArm extends LoggedSubsystem<PrototypeArmLogInputs> {
    private static PrototypeArm INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);

    private final Timer timer = new Timer();

    private final TalonSRX shoulderMotor = new TalonSRX(Ports.prototypeArmPorts.SHOULDER_MOTOR);
    private final TalonSRX elbowMotor = new TalonSRX(Ports.prototypeArmPorts.ELBOW_MOTOR);
    private final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);

    private double prevShoulderVelocity;
    private double shoulderFeedforward;
    private double prevElbowVelocity;
    private double elbowFeedForward;
    private double time;
    private double time2;
    private double shoulderVelocity;
    private double elbowVelocity;
    private double shoulderAcceleration;
    private double elbowAcceleration;

    private PrototypeArm() {
        super(new PrototypeArmLogInputs());
        timer.reset();
        timer.start();

        shoulderMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        shoulderMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMotor.setInverted(ArmConstants.clockWise);
        shoulderMotor.config_kP(0, ArmConstants.shoulderP);
        shoulderMotor.config_kI(0, ArmConstants.shoulderI);
        shoulderMotor.config_kD(0, ArmConstants.shoulderD);
        shoulderMotor.configNeutralDeadband(0.05);

        elbowMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        elbowMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.setInverted(ArmConstants.clockWise);
        elbowMotor.config_kP(0, ArmConstants.elbowP);
        elbowMotor.config_kI(0, ArmConstants.elbowI);
        elbowMotor.config_kD(0, ArmConstants.elbowD);
        elbowMotor.configNeutralDeadband(0.05);
    }

    public static PrototypeArm getInstance() {
        if (INSTANCE == null){
            INSTANCE = new PrototypeArm();
        }
        return INSTANCE;
    }

    public void setShoulderJointPower(double power) {
        shoulderMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void setElbowJointPower(double power) {
        elbowMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void setShoulderJointVelocity(double velocity) {
        shoulderMotor.set(ControlMode.Velocity, velocity);
    }

    public void setElbowJointVelocity(double velocity) {
        elbowMotor.set(ControlMode.Velocity, velocity);
    }

    public double getShoulderJointAngle() {
        return Math.toDegrees(unitModel.toUnits(shoulderMotor.getSelectedSensorPosition()));
    }

    public void setShoulderJointAngle(double angle) {
        shoulderMotor.set(ControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, shoulderFeedforward);
    }

    public double getElbowJointAngle() {
        return Math.toDegrees(unitModel.toUnits(elbowMotor.getSelectedSensorPosition()));
    }

    public void setElbowJointAngle(double angle) {
        elbowMotor.set(ControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, elbowFeedForward);
    }

    public double getShoulderMotorPower() {
        return shoulderMotor.getMotorOutputPercent();
    }

    public double getElbowMotorPower() {
        return elbowMotor.getMotorOutputPercent();
    }

    public double getShoulderMotorVelocity() {
        return unitModel.toVelocity(shoulderMotor.getSelectedSensorVelocity());
    }

    public double getElbowMotorVelocity() {
        return unitModel.toVelocity(elbowMotor.getSelectedSensorVelocity());
    }

    public void setPosition(Translation2d armLocation) {
        var angles = kinematics.inverseKinematics(armLocation.getX(), armLocation.getY());
        setShoulderJointAngle(angles.shoulderAngle);
        setElbowJointAngle(angles.elbowAngle);
    }

    public String getSubsystemName() {
        return "PrototypeArm";
    }

    public void periodic() {
        timer.reset();
        timer.start();
        shoulderVelocity = getShoulderMotorVelocity();
        elbowVelocity = getElbowMotorVelocity();
        time = timer.get();
        shoulderAcceleration = (shoulderVelocity - prevShoulderVelocity) / Math.abs(time - time2);
        elbowAcceleration = (elbowVelocity - prevElbowVelocity) / Math.abs(time - time2);
        prevShoulderVelocity = shoulderVelocity;
        prevElbowVelocity = elbowVelocity;
        time2 = timer.get();

        ArmSystemModel.ArmFeedForward tempFeedforward = systemModel.calculateFeedForward(Math.toDegrees(unitModel.toUnits(getShoulderJointAngle())), Math.toDegrees(unitModel.toUnits(getElbowJointAngle())), getShoulderMotorVelocity(), getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration);
        shoulderFeedforward = tempFeedforward.shoulderFeedForward;
        elbowFeedForward = tempFeedforward.elbowFeedForward;
    }

    public void updateInputs() {
        loggerInputs.elbowAngle = getElbowJointAngle();
        loggerInputs.shoulderAngle = getShoulderJointAngle();
        loggerInputs.elbowMotorPower = getElbowMotorPower();
        loggerInputs.shoulderMotorPower = getShoulderMotorPower();
        loggerInputs.shoulderP = ArmConstants.shoulderP;
        loggerInputs.shoulderI = ArmConstants.shoulderI;
        loggerInputs.shoulderD = ArmConstants.shoulderD;
        loggerInputs.elbowP = ArmConstants.elbowP;
        loggerInputs.elbowI = ArmConstants.elbowI;
        loggerInputs.elbowD = ArmConstants.elbowD;

    }
}
