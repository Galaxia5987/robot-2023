package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class PrototypeArm extends LoggedSubsystem<PrototypeArmLogInputs> {
    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);

    public final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

    public final TalonSRX shoulderMotor = new TalonSRX(Ports.prototypeArmPorts.SHOULDER_MOTOR);
    public final TalonSRX elbowMotor = new TalonSRX(Ports.prototypeArmPorts.ELBOW_MOTOR);
    public final CANCoder shoulderEncoder = new CANCoder(Ports.prototypeArmPorts.SHOULDER_ENCODER);
    public final CANCoder elbowEncoder = new CANCoder(Ports.prototypeArmPorts.ELBOW_ENCODER);
    public final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);

    private double prevShoulderVelocity;
    private double shoulderFeedforward;
    private double prevElbowVelocity;
    private double elbowFeedForward;
    private double Time2;

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
        shoulderMotor.set(ControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, shoulderFeedforward);
    }

    public double getElbowJointPosition() {
        return Math.toDegrees(unitModel.toUnits(elbowEncoder.getPosition()));
    }

    public void setElbowJointPosition(double angle) {
        elbowMotor.set(ControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, elbowFeedForward);
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

    public double deadBend(double value){
        if (Math.abs(value)>0.05) return value;
        return 0;
    }

    public String getSubsystemName() {
        return "PrototypeArm";
    }

    public void periodic(){
        timer.reset();
        timer.start();
        double shoulderVelocity = getShoulderMotorVelocity();
        double elbowVelocity = getElbowMotorVelocity();
        double Time = timer.get();
        double shoulderAcceleration = (shoulderVelocity - prevShoulderVelocity)/ Math.abs(Time-Time2);
        double elbowAcceleration = (elbowVelocity - prevElbowVelocity)/Math.abs(Time-Time2);
        prevShoulderVelocity = shoulderVelocity;
        prevElbowVelocity = elbowVelocity;
        Time2 = timer.get();

        ArmSystemModel.ArmFeedForward tempFeedforward = systemModel.calculateFeedForward(Math.toDegrees(unitModel.toUnits(getShoulderJointPosition())), Math.toDegrees(unitModel.toUnits(getElbowJointPosition())), getShoulderMotorVelocity(), getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration);
        shoulderFeedforward = tempFeedforward.shoulderFeedForward;
        elbowFeedForward = tempFeedforward.elbowFeedForward;
    }

    public void updateInputs() {
        loggerInputs.elbowAngle = getElbowJointPosition();
        loggerInputs.shoulderAngle = getShoulderJointPosition();
        loggerInputs.elbowMotorPower = getElbowMotorPower();
        loggerInputs.shoulderMotorPower = getShoulderMotorPower();
    }
}
