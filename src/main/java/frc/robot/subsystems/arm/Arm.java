package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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

    private final TalonFX mainShoulderMotor = new TalonFX(Ports.ArmPorts.MAIN_SHOULDER_MOTOR);
    private final TalonFX auxShoulderMotor = new TalonFX(Ports.ArmPorts.AUX_SHOULDER_MOTOR);
    private final TalonFX mainElbowMotor = new TalonFX(Ports.ArmPorts.MAIN_ELBOW_MOTOR);
    private final TalonFX auxElbowMotor = new TalonFX(Ports.ArmPorts.AUX_ELBOW_MOTOR);
    private final TalonSRX shoulderEncoder = new TalonSRX(Ports.ArmPorts.SHOULDER_ENCODER);
    private final TalonSRX elbowEncoder = new TalonSRX(Ports.ArmPorts.ELBOW_ENCODER);
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

    private Arm() {
        super(new ArmLogInputs());
        timer.reset();
        timer.start();

        mainShoulderMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        mainShoulderMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        mainShoulderMotor.setNeutralMode(NeutralMode.Brake);
        mainShoulderMotor.setInverted(ArmConstants.clockWise);
        mainShoulderMotor.config_kP(0, ArmConstants.shoulderP);
        mainShoulderMotor.config_kI(0, ArmConstants.shoulderI);
        mainShoulderMotor.config_kD(0, ArmConstants.shoulderD);
        auxShoulderMotor.follow(mainShoulderMotor);
        auxShoulderMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        auxShoulderMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        auxShoulderMotor.setNeutralMode(NeutralMode.Brake);

        mainElbowMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        mainElbowMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        mainElbowMotor.setNeutralMode(NeutralMode.Brake);
        mainElbowMotor.setInverted(ArmConstants.clockWise);
        mainElbowMotor.config_kP(0, ArmConstants.elbowP);
        mainElbowMotor.config_kI(0, ArmConstants.elbowI);
        mainElbowMotor.config_kD(0, ArmConstants.elbowD);
        auxElbowMotor.follow(mainElbowMotor);
        auxElbowMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        auxElbowMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        auxElbowMotor.setNeutralMode(NeutralMode.Brake);
    }

    public static Arm getInstance() {
        if (INSTANCE == null){
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }

    public void setShoulderJointPower(double power) {
        mainShoulderMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setElbowJointPower(double power) {
        mainElbowMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setShoulderJointVelocity(double velocity) {
        mainShoulderMotor.set(TalonFXControlMode.Velocity, velocity);
    }

    public void setElbowJointVelocity(double velocity) {
        mainElbowMotor.set(TalonFXControlMode.Velocity, velocity);
    }

    public void setShoulderJointPosition(double angle) {
        mainShoulderMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, shoulderFeedforward);
    }

    public void setElbowJointPosition(double angle) {
        mainElbowMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, elbowFeedForward);
    }

    public void setPosition(Translation2d armLocation) {
        var angles = kinematics.inverseKinematics(armLocation.getX(), armLocation.getY());
        setShoulderJointPosition(angles.shoulderAngle);
        setElbowJointPosition(angles.elbowAngle);
    }

    public double getShoulderJointPosition() {
        return Math.toDegrees(unitModel.toUnits(shoulderEncoder.getSelectedSensorPosition()));
    }

    public double getElbowJointPosition() {
        return Math.toDegrees(unitModel.toUnits(elbowEncoder.getSelectedSensorPosition()));
    }

    public double getShoulderMotorPower() {
        return mainShoulderMotor.getMotorOutputPercent();
    }

    public double getElbowMotorPower() {
        return mainElbowMotor.getMotorOutputPercent();
    }

    public double getShoulderMotorVelocity() {
        return unitModel.toVelocity(mainShoulderMotor.getSelectedSensorVelocity());
    }

    public double getElbowMotorVelocity() {
        return unitModel.toVelocity(mainElbowMotor.getSelectedSensorVelocity());
    }

    public double deadBend(double value) {
        if (Math.abs(value) > 0.05) return value;
        return 0;
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

        ArmSystemModel.ArmFeedForward tempFeedforward = systemModel.calculateFeedForward(Math.toDegrees(unitModel.toUnits(getShoulderJointPosition())), Math.toDegrees(unitModel.toUnits(getElbowJointPosition())), getShoulderMotorVelocity(), getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration);
        shoulderFeedforward = tempFeedforward.shoulderFeedForward;
        elbowFeedForward = tempFeedforward.elbowFeedForward;
    }

    public void updateInputs() {
        loggerInputs.elbowAngle = getElbowJointPosition();
        loggerInputs.shoulderAngle = getShoulderJointPosition();
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
