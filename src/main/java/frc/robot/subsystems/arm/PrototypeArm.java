package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class PrototypeArm extends LoggedSubsystem<PrototypeArmLogInputs> {
    public static final TalonSRX shoulderMotor = new TalonSRX(Ports.prototypeArmPorts.SHOULDER_MOTOR);
    public static final TalonSRX elbowMotor = new TalonSRX(Ports.prototypeArmPorts.ELBOW_MOTOR);
    public static final CANCoder shoulderEncoder = new CANCoder(Ports.prototypeArmPorts.SHOULDER_ENCODER);
    public static final CANCoder elbowEncoder = new CANCoder(Ports.prototypeArmPorts.ELBOW_ENCODER);
    public static final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);
    public static PrototypeArm INSTANCE = null;

    public PrototypeArm() {
        super(new PrototypeArmLogInputs());

        shoulderMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        shoulderMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPANSATION);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMotor.setInverted(ArmConstants.clockWise);

        elbowMotor.configVoltageCompSaturation(ArmConstants.CONFIG_VOLT_COMP);
        elbowMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPANSATION);
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.setInverted(ArmConstants.clockWise);
    }

    public void setShoulderJointPower(double power) {
        shoulderMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void setElbowJointPower(double power) {
        elbowMotor.set(TalonSRXControlMode.PercentOutput, power);
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

    public String getSubsystemName() {
        return "PrototypeArm";
    }

    public void updateInputs() {

    }
}
