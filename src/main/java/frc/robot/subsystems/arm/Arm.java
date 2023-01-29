package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class Arm extends LoggedSubsystem<ArmLogInputs> {
    private static Arm INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);
    private final ArmAccelerationCalculation accelerationCalculation = new ArmAccelerationCalculation();
    private final TalonFX shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);
    private final TalonFX shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);
    private final TalonFX elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);
    private final TalonFX elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);
    private final TalonSRX shoulderEncoder = new TalonSRX(Ports.ArmPorts.SHOULDER_ENCODER);
    //private final FeedbackDevice shoulderEncoder = new FeedbackDevice();
    private final TalonSRX elbowEncoder = new TalonSRX(Ports.ArmPorts.ELBOW_ENCODER);
    private final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);
    private double shoulderFeedforward;
    private double elbowFeedForward;
    private double shoulderAcceleration;
    private double elbowAcceleration;
    private double shoulderSetPoint;
    private double elbowSetPoint;

    private Arm() {
        super(new ArmLogInputs());
        shoulderMainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        shoulderMainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        shoulderMainMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMainMotor.setInverted(ArmConstants.clockWise);
        shoulderMainMotor.config_kP(0, ArmConstants.shoulderP);
        shoulderMainMotor.config_kI(0, ArmConstants.shoulderI);
        shoulderMainMotor.config_kD(0, ArmConstants.shoulderD);
        shoulderMainMotor.configNeutralDeadband(ArmConstants.DEADBAND);
        //shoulderMainMotor.configSelectedFeedbackSensor(new FeedbackDevice());
        shoulderAuxMotor.follow(shoulderMainMotor);
        shoulderAuxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        shoulderAuxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        shoulderAuxMotor.setNeutralMode(NeutralMode.Brake);

        elbowMainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        elbowMainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        elbowMainMotor.setNeutralMode(NeutralMode.Brake);
        elbowMainMotor.setInverted(ArmConstants.clockWise);
        elbowMainMotor.config_kP(0, ArmConstants.elbowP);
        elbowMainMotor.config_kI(0, ArmConstants.elbowI);
        elbowMainMotor.config_kD(0, ArmConstants.elbowD);
        elbowMainMotor.configNeutralDeadband(ArmConstants.DEADBAND);
        elbowAuxMotor.follow(elbowMainMotor);
        elbowAuxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        elbowAuxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        elbowAuxMotor.setNeutralMode(NeutralMode.Brake);
    }

    public static Arm getInstance() {
        if (INSTANCE == null) {
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

    public void setPosition(Translation2d armLocation) {
        var angles = kinematics.inverseKinematics(armLocation.getX(), armLocation.getY());
        setShoulderJointAngle(angles.shoulderAngle);
        setElbowJointAngle(angles.elbowAngle);
    }

    public void setShoulderJointAngle(double angle) {
        shoulderMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, shoulderFeedforward);
        shoulderSetPoint = angle;
    }

    public double getShoulderJointAngle() {
        return unitModel.toUnits(shoulderEncoder.getSelectedSensorPosition());
    }

    public double getElbowJointAngle() {
        return unitModel.toUnits(elbowEncoder.getSelectedSensorPosition());
    }

    public Translation2d getPosition(){
        return kinematics.forwardKinematics(getShoulderJointAngle(), getElbowJointAngle());
    }

    public void setElbowJointAngle(double angle) {
        elbowMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)), DemandType.ArbitraryFeedForward, elbowFeedForward);
        elbowSetPoint = angle;
    }

    public double getShoulderMotorVelocity() {
        return unitModel.toVelocity(shoulderMainMotor.getSelectedSensorVelocity());
    }

    public double getElbowMotorVelocity() {
        return unitModel.toVelocity(elbowMainMotor.getSelectedSensorVelocity());
    }

    public String getSubsystemName() {
        return "Arm";
    }

    public void periodic() {
        shoulderAcceleration = accelerationCalculation.getArmAcceleration(JointType.SHOULDER.value);
        elbowAcceleration = accelerationCalculation.getArmAcceleration(JointType.ELBOW.value);

        ArmSystemModel.ArmFeedForward tempFeedforward = systemModel.calculateFeedForward(Math.toDegrees(unitModel.toUnits(getShoulderJointAngle())), Math.toDegrees(unitModel.toUnits(getElbowJointAngle())), getShoulderMotorVelocity(), getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration);
        shoulderFeedforward = tempFeedforward.shoulderFeedForward;
        elbowFeedForward = tempFeedforward.elbowFeedForward;
    }

    public void updateInputs() {
        loggerInputs.shoulderAngle = getShoulderJointAngle();
        loggerInputs.elbowAngle = getElbowJointAngle();
        loggerInputs.shoulderMotorPower = shoulderMainMotor.getMotorOutputPercent();
        loggerInputs.elbowMotorPower = elbowMainMotor.getMotorOutputPercent();
        loggerInputs.shoulderSetPoint = shoulderSetPoint;
        loggerInputs.elbowSetPoint = elbowSetPoint;
    }

    public enum JointType{
        SHOULDER(true),
        ELBOW(false);

        public final boolean value;

        JointType(boolean value) {
            this.value = value;
        }
    }
}
