package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.math.AngleUtil;
import frc.robot.utils.units.UnitModel;

public class Arm extends LoggedSubsystem<ArmInputsAutoLogged> {
    private static Arm INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);

    private final TalonFX shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);
    private final TalonFX shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);
    private final DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(Ports.ArmPorts.SHOULDER_ENCODER);
    private final LinearFilter shoulderEncoderFilter = LinearFilter.movingAverage(10);

    private final TalonFX elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);
    private final TalonFX elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);
    private final DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(Ports.ArmPorts.ELBOW_ENCODER);

    private final UnitModel unitModelShoulder = new UnitModel(ArmConstants.TICKS_PER_RADIAN_SHOULDER);
    private final UnitModel unitModelElbow = new UnitModel(ArmConstants.TICKS_PER_RADIAN_ELBOW);

    private double shoulderOffset = ArmConstants.SHOULDER_ABSOLUTE_ENCODER_OFFSET;
    private double elbowOffset = ArmConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET;

    private double shoulderFeedforward;
    private double elbowFeedforward;

    private Arm() {
        super(new ArmInputsAutoLogged());
        shoulderMainMotor.configFactoryDefault();
        shoulderAuxMotor.configFactoryDefault();
        elbowMainMotor.configFactoryDefault();
        elbowAuxMotor.configFactoryDefault();

        shoulderMainMotor.setInverted(TalonFXInvertType.CounterClockwise);
        elbowMainMotor.setInverted(ArmConstants.MAIN_CLOCKWISE);

        shoulderMainMotor.configMotionAcceleration(unitModelShoulder.toTicks100ms(ArmConstants.SHOULDER_ACCELERATION));
        elbowMainMotor.configMotionCruiseVelocity(unitModelElbow.toTicks100ms(ArmConstants.ELBOW_ACCELERATION));
        shoulderMainMotor.configMotionAcceleration(unitModelShoulder.toTicks100ms(ArmConstants.SHOULDER_CRUISE_VELOCITY));
        elbowMainMotor.configMotionCruiseVelocity(unitModelElbow.toTicks100ms(ArmConstants.ELBOW_CRUISE_VELOCITY));

        configureMainMotor(shoulderMainMotor, ArmConstants.shoulderP, ArmConstants.shoulderI, ArmConstants.shoulderD);
        configureAuxMotor(shoulderAuxMotor, shoulderMainMotor);

        configureMainMotor(elbowMainMotor, ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD);
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
     * Configures the aux motors.
     */
    private void configureAuxMotor(TalonFX auxMotor, TalonFX mainMotor) {
        auxMotor.follow(mainMotor);
        auxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        auxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_SATURATION);
        auxMotor.setNeutralMode(NeutralMode.Brake);
        auxMotor.setInverted(TalonFXInvertType.FollowMaster);
    }

    /**
     * Configures the main motors.
     */
    private void configureMainMotor(TalonFX mainMotor, double kP, double kI, double kD) {
        mainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        mainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_SATURATION);
        mainMotor.setNeutralMode(NeutralMode.Brake);

        mainMotor.config_kP(0, kP);
        mainMotor.config_kI(0, kI);
        mainMotor.config_kD(0, kD);
    }

    /**
     * Sets the power of the shoulder motors.
     *
     * @param power desired power. [-1,1]
     */
    public void setShoulderJointPower(double power) {
        shoulderMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets the power of the elbow motors.
     *
     * @param power desired power. [-1,1]
     */
    public void setElbowJointPower(double power) {
        elbowMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Gets the angle of the shoulder joint.
     *
     * @return angle of the shoulder joint. [rad]
     */
    public Rotation2d getShoulderJointAngle() {
        return Rotation2d.fromRotations(shoulderEncoderFilter.calculate(shoulderEncoder.getAbsolutePosition() - shoulderOffset));
    }

    /**
     * Sets the angle of the shoulder joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setShoulderJointAngle(double angle) {
        double error = unitModelShoulder.toTicks(Math.toRadians(angle)) - unitModelShoulder.toTicks(getShoulderJointAngle().getRadians());
        shoulderMainMotor.set(TalonFXControlMode.Position, shoulderMainMotor.getSelectedSensorPosition() + error,
                DemandType.ArbitraryFeedForward, shoulderFeedforward);
        loggerInputs.shoulderSetpoint = angle;
        loggerInputs.shoulderError = error;
    }

    /**
     * Gets the angle of the elbow joint.
     *
     * @return elbow joint angle. [rad]
     */
    public Rotation2d getElbowJointAngle() {
        return Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition() - elbowOffset);
    }

    /**
     * Sets the angle of the elbow joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setElbowJointAngle(double angle) {
        double error = unitModelElbow.toTicks(Math.toRadians(angle)) - unitModelElbow.toTicks(getElbowJointAngle().getRadians());
        elbowMainMotor.set(TalonFXControlMode.Position, elbowMainMotor.getSelectedSensorPosition() + error,
                DemandType.ArbitraryFeedForward, elbowFeedforward);
        loggerInputs.elbowSetpoint = angle;
        loggerInputs.elbowError = error;
    }

    /**
     * Calculates the position of the end of the arm.
     *
     * @return Translation2d of the position.
     */
    public Translation2d getEndPosition() {
        double shoulderAngle = getShoulderJointAngle().getRadians();
        double elbowAngle = getElbowJointAngle().getRadians();
        return kinematics.forwardKinematics(shoulderAngle, shoulderAngle + elbowAngle - Math.PI);
    }

    /**
     * Sets the position of the end of the arm.
     *
     * @param armLocation Translation2d of the desired location.
     */
    public void setEndPosition(Translation2d armLocation) {
        var angles = kinematics.inverseKinematics(armLocation);
        setShoulderJointAngle(angles.shoulderAngle);
        setElbowJointAngle(angles.elbowAngle);
    }

    /**
     * Gets the velocity of the shoulder motors.
     *
     * @return shoulder motors velocity. [rad/sec]
     */
    public double getShoulderMotorVelocity() {
        return unitModelShoulder.toVelocity(shoulderMainMotor.getSelectedSensorVelocity());
    }

    /**
     * Gets the velocity of the elbow motors.
     *
     * @return elbow motor velocity. [rad/sec]
     */
    public double getElbowMotorVelocity() {
        return unitModelElbow.toVelocity(elbowMainMotor.getSelectedSensorVelocity());
    }

    public void resetArmEncoders() {
        double elbowAngleReset = elbowEncoder.getAbsolutePosition() * 360.0 - ArmConstants.ELBOW_ZERO_POSITION;
        double shoulderAngleReset = shoulderEncoder.getAbsolutePosition() * 360.0 - ArmConstants.SHOULDER_ZERO_POSITION;
        shoulderOffset = AngleUtil.normalize(shoulderAngleReset) / 360.0;
        elbowOffset = AngleUtil.normalize(elbowAngleReset) / 360.0;
    }

    /**
     * Stops the motors.
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
        SmartDashboard.putString("Arm Encoder Offset", "{" + shoulderOffset + ", " + elbowOffset + "}");
        Translation2d position = getEndPosition();
        Rotation2d angle = new Rotation2d(position.getX(), position.getY());
        shoulderFeedforward = ArmConstants.SHOULDER_FEED_FORWARD *
                position.getNorm() * angle.getCos();
        elbowFeedforward = ArmConstants.EBLOW_FEED_FORWARD *
                getElbowJointAngle()
                .plus(getShoulderJointAngle())
                .minus(Rotation2d.fromDegrees(180))
                .getCos() * ArmConstants.ELBOW_ARM_LENGTH;
    }

    @Override
    public void updateInputs() {
        loggerInputs.shoulderAngle = getShoulderJointAngle().getDegrees();
        loggerInputs.elbowAngle = getElbowJointAngle().getDegrees();
        loggerInputs.shoulderMotorPower = shoulderMainMotor.getMotorOutputPercent();
        loggerInputs.elbowMotorPower = elbowMainMotor.getMotorOutputPercent();
        loggerInputs.shoulderEncoderPosition = shoulderEncoder.getAbsolutePosition();
        loggerInputs.elbowEncoderPosition = elbowEncoder.getAbsolutePosition();
        loggerInputs.shoulderVelocity = getShoulderMotorVelocity();
        loggerInputs.elbowVelocity = getElbowMotorVelocity();
        Translation2d position = getEndPosition();
        loggerInputs.armPosition[0] = position.getX();
        loggerInputs.armPosition[1] = position.getY();
        ArmKinematics.InverseKinematicsSolution kinematicsSolution = kinematics.inverseKinematics(position);
        loggerInputs.inverseKinematicsSolution[0] = Math.toDegrees(kinematicsSolution.shoulderAngle);
        loggerInputs.inverseKinematicsSolution[1] = Math.toDegrees(kinematicsSolution.elbowAngle);

        loggerInputs.feedforward[0] = shoulderFeedforward;
        loggerInputs.feedforward[1] = elbowFeedforward;

        loggerInputs.shoulderOutputVoltage = shoulderMainMotor.getMotorOutputVoltage();
    }
}
