package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class Arm extends LoggedSubsystem<ArmLogInputs> {
    private static Arm INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);

    private final Timer timer = new Timer();

    private final TalonFX shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);
    private final TalonFX shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);
    private final TalonFX elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);
    private final TalonFX elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);
    private final TalonSRX shoulderEncoder = new TalonSRX(Ports.ArmPorts.SHOULDER_ENCODER);
    private final TalonSRX elbowEncoder = new TalonSRX(Ports.ArmPorts.ELBOW_ENCODER);
    private final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);

    private double prevShoulderVelocity;
    private double shoulderFeedforward;
    private double prevElbowVelocity;
    private double elbowFeedForward;
    private double time;
    private double time2 = 0;
    private double shoulderVelocity;
    private double elbowVelocity;
    private double shoulderAcceleration;
    private double elbowAcceleration;

    private Arm() {
        super(new ArmLogInputs());
        timer.reset();
        timer.start();

        shoulderMainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        shoulderMainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        shoulderMainMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMainMotor.setInverted(ArmConstants.clockWise);
        shoulderMainMotor.config_kP(0, ArmConstants.shoulderP);
        shoulderMainMotor.config_kI(0, ArmConstants.shoulderI);
        shoulderMainMotor.config_kD(0, ArmConstants.shoulderD);
        shoulderMainMotor.configNeutralDeadband(ArmConstants.DEAD_BEND);
        shoulderAuxMotor.follow(shoulderMainMotor);
        shoulderAuxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        shoulderAuxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        shoulderAuxMotor.setNeutralMode(NeutralMode.Brake);

        elbowMainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        elbowMainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        elbowMainMotor.setNeutralMode(NeutralMode.Brake);
        elbowMainMotor.setInverted(ArmConstants.clockWise);
        elbowMainMotor.config_kP(0, ArmConstants.elbowP);
        elbowMainMotor.config_kI(0, ArmConstants.elbowI);
        elbowMainMotor.config_kD(0, ArmConstants.elbowD);
        elbowMainMotor.configNeutralDeadband(ArmConstants.DEAD_BEND);
        elbowAuxMotor.follow(elbowMainMotor);
        elbowAuxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        elbowAuxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        elbowAuxMotor.setNeutralMode(NeutralMode.Brake);
    }

    public static Arm getInstance() {
        if (INSTANCE == null){
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }

    public void setShoulderJointPower(double power) {
        shoulderMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setElbowJointPower(double power) {
        elbowMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setShoulderJointAngle(double angle) {
        shoulderMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, shoulderFeedforward);
    }

    public void setElbowJointAngle(double angle) {
        elbowMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, elbowFeedForward);
    }

    public void setPosition(Translation2d armLocation) {
        var angles = kinematics.inverseKinematics(armLocation.getX(), armLocation.getY());
        setShoulderJointAngle(angles.shoulderAngle);
        setElbowJointAngle(angles.elbowAngle);
    }

    public double getShoulderJointAngle() {
        return unitModel.toUnits(shoulderEncoder.getSelectedSensorPosition());
    }

    public double getElbowJointAngle() {
        return unitModel.toUnits(elbowEncoder.getSelectedSensorPosition());
    }

    public double getShoulderMotorPower() {
        return shoulderMainMotor.getMotorOutputPercent();
    }

    public double getElbowMotorPower() {
        return elbowMainMotor.getMotorOutputPercent();
    }

    public double getShoulderMotorVelocity() {
        return unitModel.toVelocity(shoulderMainMotor.getSelectedSensorVelocity());
    }

    public double getElbowMotorVelocity() {
        return unitModel.toVelocity(elbowMainMotor.getSelectedSensorVelocity());
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
    }
}
