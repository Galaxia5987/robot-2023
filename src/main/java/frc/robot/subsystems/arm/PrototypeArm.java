package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class PrototypeArm extends LoggedSubsystem<PrototypeArmLogInputs> {
    public final Timer timer = new Timer();
    public final TalonSRX shoulderMotor = new TalonSRX(Ports.prototypeArmPorts.SHOULDER_MOTOR);
    public final TalonSRX elbowMotor = new TalonSRX(Ports.prototypeArmPorts.ELBOW_MOTOR);
    public final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);
    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);
    private double prevShoulderVelocity;
    private double shoulderFeedforward;
    private double prevElbowVelocity;
    private double elbowFeedForward;
    private double Time2;

    public PrototypeArm() {
        super(new PrototypeArmLogInputs());
        timer.reset();
        timer.start();

//        shoulderMotor.configFactoryDefault();
//        elbowMotor.configFactoryDefault();

        shoulderMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        shoulderMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPANSATION);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMotor.setInverted(ArmConstants.clockWise);
        shoulderMotor.config_kP(0, ArmConstants.shoulderP);
        shoulderMotor.config_kI(0, ArmConstants.shoulderI);
        shoulderMotor.config_kD(0, ArmConstants.shoulderD);
        shoulderMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);

        elbowMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        elbowMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPANSATION);
        elbowMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);

        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.setInverted(false);
        elbowMotor.setSensorPhase(true);
        elbowMotor.config_kP(0, ArmConstants.elbowP);
        elbowMotor.config_kI(0, ArmConstants.elbowI);
        elbowMotor.config_kD(0, ArmConstants.elbowD);

        elbowMotor.configMotionAcceleration(unitModel.toTicks100ms(Math.PI / 5));
        elbowMotor.configMotionCruiseVelocity(unitModel.toTicks100ms(Math.PI / 5));
    }

    public void setShoulderJointPower(double power) {
        shoulderMotor.set(ControlMode.PercentOutput, power);
    }

    public void setElbowJointPower(double power) {
        elbowMotor.set(ControlMode.PercentOutput, power);
    }

    public void setShoulderJointVelocity(double velocity) {
        shoulderMotor.set(ControlMode.Velocity, velocity);
    }

    public void setElbowJointVelocity(double velocity) {
        elbowMotor.set(ControlMode.Velocity, velocity);
    }

    public double getShoulderJointPosition() {
        return Math.toDegrees(unitModel.toUnits(shoulderMotor.getSelectedSensorPosition()));
    }

    public void setShoulderJointPosition(double angle) {
        double error = Rotation2d.fromDegrees(getShoulderJointPosition()).minus(Rotation2d.fromDegrees(angle)).getRadians();
        shoulderMotor.set(ControlMode.MotionMagic, shoulderMotor.getSelectedSensorPosition() + unitModel.toTicks(error),
                DemandType.ArbitraryFeedForward, shoulderFeedforward);
    }

    public double getElbowJointPosition() {
        return Math.toDegrees(unitModel.toUnits(elbowMotor.getSelectedSensorPosition())) - ArmConstants.ZERO_ANGLE;
    }

    public void setElbowJointPosition(double angle) {
        double error = Rotation2d.fromDegrees(getElbowJointPosition()).minus(Rotation2d.fromDegrees(angle)).getRadians();
        elbowMotor.set(ControlMode.MotionMagic, elbowMotor.getSelectedSensorPosition() + unitModel.toTicks(error),
                DemandType.ArbitraryFeedForward, elbowFeedForward);
    }

    public double getShoulderMotorPower() {
        return shoulderMotor.getMotorOutputPercent();
    }

    public double getElbowMotorPower() {
        return elbowMotor.getMotorOutputPercent();
    }

    public double getShoulderMotorVelocity() {
        return shoulderMotor.getSelectedSensorVelocity();
    }

    public double getElbowMotorVelocity() {
        return elbowMotor.getSelectedSensorVelocity();
    }

    public void setPosition(Translation2d armLocation) {
        setShoulderJointPosition(kinematics.inverseKinematics(armLocation.getX(), armLocation.getY()).shoulderAngle);
        setElbowJointPosition(kinematics.inverseKinematics(armLocation.getX(), armLocation.getY()).elbowAngle);
    }
    @Override
    public String getSubsystemName() {
        return "PrototypeArm";
    }

    @Override
    public void periodic() {
        timer.reset();
        timer.start();
        double shoulderVelocity = getShoulderMotorVelocity();
        double elbowVelocity = getElbowMotorVelocity();
        double Time = timer.get();
        double shoulderAcceleration = (shoulderVelocity - prevShoulderVelocity) / Math.abs(Time - Time2);
        double elbowAcceleration = (elbowVelocity - prevElbowVelocity) / Math.abs(Time - Time2);
        prevShoulderVelocity = shoulderVelocity;
        prevElbowVelocity = elbowVelocity;
        Time2 = timer.get();

        ArmSystemModel.ArmFeedForward tempFeedforward = systemModel.calculateFeedForward(Math.toDegrees(unitModel.toUnits(getShoulderJointPosition())), Math.toDegrees(unitModel.toUnits(getElbowJointPosition())), getShoulderMotorVelocity(), getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration);
        shoulderFeedforward = tempFeedforward.shoulderFeedForward;
        elbowFeedForward = tempFeedforward.elbowFeedForward;
    }

    @Override
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
