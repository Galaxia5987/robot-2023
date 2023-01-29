package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;
import edu.wpi.first.wpilibj.Timer;

public class Arm extends LoggedSubsystem<ArmLogInputs> {
    private static Arm INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);
    private final ArmAccelerationCalculation shoulderAccelerationCalculation = new ArmAccelerationCalculation();
    private final ArmAccelerationCalculation elbowAccelerationCalculation = new ArmAccelerationCalculation();

    private final TalonFX shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);
    private final TalonFX shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);
    private final CANCoder shoulderEncoder = new CANCoder(Ports.ArmPorts.SHOULDER_ENCODER);

    private final TalonFX elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);
    private final TalonFX elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);
    private final CANCoder elbowEncoder = new CANCoder(Ports.ArmPorts.ELBOW_ENCODER);

    private final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);
    private double shoulderFeedforward;
    private double elbowFeedForward;
    private double shoulderSetPoint;
    private double elbowSetPoint;

    private Arm() {
        super(new ArmLogInputs());
        configureMainMotor(shoulderMainMotor, ArmConstants.shoulderP, ArmConstants.shoulderI, ArmConstants.shoulderD, shoulderEncoder);
        configureAuxMotor(shoulderAuxMotor, shoulderMainMotor);

        configureMainMotor(elbowMainMotor, ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD, elbowEncoder);
        configureAuxMotor(elbowAuxMotor, elbowMainMotor);
    }

    public static Arm getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }

    private void configureAuxMotor(TalonFX shoulderAuxMotor, TalonFX shoulderMainMotor) {
        shoulderAuxMotor.follow(shoulderMainMotor);
        shoulderAuxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        shoulderAuxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        shoulderAuxMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void configureMainMotor(TalonFX shoulderMainMotor, double shoulderP, double shoulderI, double shoulderD, CANCoder shoulderEncoder) {
        shoulderMainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        shoulderMainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_Saturation);
        shoulderMainMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMainMotor.setInverted(ArmConstants.clockWise);
        shoulderMainMotor.config_kP(0, shoulderP);
        shoulderMainMotor.config_kI(0, shoulderI);
        shoulderMainMotor.config_kD(0, shoulderD);
        shoulderMainMotor.configNeutralDeadband(ArmConstants.DEADBAND);
        shoulderMainMotor.setSelectedSensorPosition(unitModel.toTicks(shoulderEncoder.getAbsolutePosition()));
    }

    public void setShoulderJointPower(double power) {
        shoulderMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setElbowJointPower(double power) {
        elbowMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public double getShoulderJointAngle() {
        return unitModel.toUnits(shoulderMainMotor.getSelectedSensorPosition());
    }

    public void setShoulderJointAngle(double angle) {
        shoulderMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)),
                DemandType.ArbitraryFeedForward, shoulderFeedforward);
        shoulderSetPoint = angle;
    }

    public double getElbowJointAngle() {
        return unitModel.toUnits(elbowMainMotor.getSelectedSensorPosition());
    }

    public void setElbowJointAngle(double angle) {
        elbowMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)),
                DemandType.ArbitraryFeedForward, elbowFeedForward);
        elbowSetPoint = angle;
    }

    public Translation2d getEndPosition() {
        return kinematics.forwardKinematics(getShoulderJointAngle(), getElbowJointAngle());
    }

    public void setEndPosition(Translation2d armLocation) {
        var angles = kinematics.inverseKinematics(armLocation.getX(), armLocation.getY());
        setShoulderJointAngle(angles.shoulderAngle);
        setElbowJointAngle(angles.elbowAngle);
    }

    public double getShoulderMotorVelocity() {
        return unitModel.toVelocity(shoulderMainMotor.getSelectedSensorVelocity());
    }

    public double getElbowMotorVelocity() {
        return unitModel.toVelocity(elbowMainMotor.getSelectedSensorVelocity());
    }

    @Override
    public String getSubsystemName() {
        return "Arm";
    }

    @Override
    public void periodic() {
        shoulderAccelerationCalculation.addVelocity(getShoulderMotorVelocity(), Timer.getFPGATimestamp());
        double shoulderAcceleration = shoulderAccelerationCalculation.getAcceleration();
        elbowAccelerationCalculation.addVelocity(getElbowMotorVelocity(), Timer.getFPGATimestamp());
        double elbowAcceleration = elbowAccelerationCalculation.getAcceleration();

        ArmSystemModel.ArmFeedForward tempFeedforward = systemModel.calculateFeedForward(
                unitModel.toUnits(getShoulderJointAngle()),
                Math.toDegrees(unitModel.toUnits(getElbowJointAngle())), getShoulderMotorVelocity(),
                getElbowMotorVelocity(), shoulderAcceleration, elbowAcceleration
        );
        shoulderFeedforward = tempFeedforward.shoulderFeedForward;
        elbowFeedForward = tempFeedforward.elbowFeedForward;
    }

    @Override
    public void updateInputs() {
        loggerInputs.shoulderAngle = getShoulderJointAngle();
        loggerInputs.elbowAngle = getElbowJointAngle();
        loggerInputs.shoulderMotorPower = shoulderMainMotor.getMotorOutputPercent();
        loggerInputs.elbowMotorPower = elbowMainMotor.getMotorOutputPercent();
        loggerInputs.shoulderSetPoint = shoulderSetPoint;
        loggerInputs.elbowSetPoint = elbowSetPoint;
    }

    public enum JointType {
        SHOULDER(true),
        ELBOW(false);

        public final boolean value;

        JointType(boolean value) {
            this.value = value;
        }
    }
}
