package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class Arm extends LoggedSubsystem<ArmInputs> {
    private static Arm INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);
    private final ArmSystemModel systemModel = new ArmSystemModel(ArmConstants.ARM_CONSTANTS);
    private final AccelerationCalculation shoulderAccelerationCalculation = new AccelerationCalculation();
    private final AccelerationCalculation elbowAccelerationCalculation = new AccelerationCalculation();

    private final TalonFX shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);
    private final TalonFX shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);
    private final CANCoder shoulderEncoder = new CANCoder(Ports.ArmPorts.SHOULDER_ENCODER);

    private final TalonFX elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);
    private final TalonFX elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);
    private final CANCoder elbowEncoder = new CANCoder(Ports.ArmPorts.ELBOW_ENCODER);

    private final UnitModel unitModel = new UnitModel(ArmConstants.TICKS_PER_RADIAN);
    private double shoulderFeedforward;
    private double elbowFeedForward;
    private double shoulderSetpoint;
    private double elbowSetpoint;

    private Arm() {
        super(new ArmInputs());
        configureMainMotor(shoulderMainMotor, ArmConstants.shoulderP, ArmConstants.shoulderI, ArmConstants.shoulderD, shoulderEncoder);
        configureAuxMotor(shoulderAuxMotor, shoulderMainMotor);

        configureMainMotor(elbowMainMotor, ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD, elbowEncoder);
        configureAuxMotor(elbowAuxMotor, elbowMainMotor);
    }

    /**
     * Get the instance of the arm subsystem.
     *
     * @return Arm instance
     */
    public static Arm getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }

    /**
     * Configures the aux motors
     *
     * @param auxMotor
     * @param mainMotor
     */
    private void configureAuxMotor(TalonFX auxMotor, TalonFX mainMotor) {
        auxMotor.follow(mainMotor);
        auxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        auxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_SATURATION);
        auxMotor.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION);
        auxMotor.configMotionCruiseVelocity(ArmConstants.MOTION_CRUISE_VELOCITY);
        auxMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Configures the main motors
     *
     * @param mainMotor
     * @param kP
     * @param kI
     * @param kD
     * @param encoder
     */
    private void configureMainMotor(TalonFX mainMotor, double kP, double kI, double kD, CANCoder encoder) {
        mainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        mainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_SATURATION);
        mainMotor.setNeutralMode(NeutralMode.Brake);
        mainMotor.setInverted(ArmConstants.CLOCKWISE);

        mainMotor.config_kP(0, kP);
        mainMotor.config_kI(0, kI);
        mainMotor.config_kD(0, kD);

        mainMotor.configNeutralDeadband(ArmConstants.DEADBAND);
        mainMotor.setSelectedSensorPosition(unitModel.toTicks(encoder.getAbsolutePosition()));
        mainMotor.configMotionAcceleration(ArmConstants.MOTION_ACCELERATION);
        mainMotor.configMotionCruiseVelocity(ArmConstants.MOTION_CRUISE_VELOCITY);
    }

    /**
     * Sets the power of the shoulder motors
     *
     * @param power desired power
     */
    public void setShoulderJointPower(double power) {
        shoulderMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets the power of the elbow motors
     *
     * @param power desired power
     */
    public void setElbowJointPower(double power) {
        elbowMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Gets the angle of the shoulder joint
     *
     * @return angle of the shoulder joint [rad]
     */
    public double getShoulderJointAngle() {
        return unitModel.toUnits(shoulderMainMotor.getSelectedSensorPosition());
    }

    /**
     * Sets the angle of the shoulder joint
     *
     * @param angle desired angle [degrees]
     */
    public void setShoulderJointAngle(double angle) {
        shoulderMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)),
                DemandType.ArbitraryFeedForward, shoulderFeedforward);
        shoulderSetpoint = angle;
    }

    /**
     * Gets the angle of the elbow joint
     *
     * @return elbow joint angle [rad]
     */
    public double getElbowJointAngle() {
        return unitModel.toUnits(elbowMainMotor.getSelectedSensorPosition());
    }

    /**
     * Sets the angle of the elbow joint
     *
     * @param angle desired angle [degrees]
     */
    public void setElbowJointAngle(double angle) {
        elbowMainMotor.set(TalonFXControlMode.MotionMagic, unitModel.toTicks(Math.toRadians(angle)),
                DemandType.ArbitraryFeedForward, elbowFeedForward);
        elbowSetpoint = angle;
    }

    /**
     * Calculates the position of the end of the arm
     *
     * @return Translation2d of the position
     */
    public Translation2d getEndPosition() {
        return kinematics.forwardKinematics(getShoulderJointAngle(), getElbowJointAngle());
    }

    /**
     * Sets the position of the end of the arm
     *
     * @param armLocation Translation2d of the desired location
     */
    public void setEndPosition(Translation2d armLocation) {
        var angles = kinematics.inverseKinematics(armLocation);
        setShoulderJointAngle(angles.shoulderAngle);
        setElbowJointAngle(angles.elbowAngle);
    }

    /**
     * Gets the velocity of the shoulder motors
     *
     * @return shoulder motors velocity
     */
    public double getShoulderMotorVelocity() {
        return unitModel.toVelocity(shoulderMainMotor.getSelectedSensorVelocity());
    }

    /**
     * Gets the velocity of the elbow motors
     *
     * @return elbow motor velocity
     */
    public double getElbowMotorVelocity() {
        return unitModel.toVelocity(elbowMainMotor.getSelectedSensorVelocity());
    }

    /**
     * Stops the motors
     */
    public void stop() {
        shoulderMainMotor.neutralOutput();
        elbowMainMotor.neutralOutput();
    }

    @Override
    public String getSubsystemName() {
        return "Arm";
    }

    @Override
    public void periodic() {
        shoulderAccelerationCalculation.addVelocity(getShoulderMotorVelocity(), Timer.getFPGATimestamp());
        double shoulderAcceleration = shoulderAccelerationCalculation.getAcceleration(); // rad/sec^2

        elbowAccelerationCalculation.addVelocity(getElbowMotorVelocity(), Timer.getFPGATimestamp());
        double elbowAcceleration = elbowAccelerationCalculation.getAcceleration(); // rad/sec^2

        var tempFeedforward = systemModel.calculateFeedForward(
                getShoulderJointAngle(), getElbowJointAngle(),
                getShoulderMotorVelocity(), getElbowMotorVelocity(),
                shoulderAcceleration, elbowAcceleration
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
        loggerInputs.shoulderSetpoint = shoulderSetpoint;
        loggerInputs.shoulderSetpoint = elbowSetpoint;
    }
}
