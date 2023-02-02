package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.motors.PIDTalon;

import static frc.robot.Constants.TALON_TIMEOUT;
import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class SwerveModule extends LoggedSubsystem<SwerveModuleLogInputs> {
    private final TalonFX driveMotor;
    private final PIDTalon angleMotor;
    private final DutyCycleEncoder encoder;
    private final int offset;
    private final SwerveDrive.Module number;
    private final double[] motionMagicConfigs;
    private boolean initializedOffset = false;

    public SwerveModule(SwerveDrive.Module number, int driveMotorPort, int angleMotorPort, int encoderPort, int offset, boolean driveInverted,
                        boolean angleInverted, boolean angleSensorPhase, double[] motionMagicConfigs) {
        super(new SwerveModuleLogInputs());
        this.number = number;
        this.offset = offset;
        this.motionMagicConfigs = motionMagicConfigs;
        driveMotor = new TalonFX(driveMotorPort);
        angleMotor = new PIDTalon(angleMotorPort);

        driveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TALON_TIMEOUT);
        driveMotor.setInverted(driveInverted);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.configNeutralDeadband(0.175);
        driveMotor.setSelectedSensorPosition(0);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(false, TALON_TIMEOUT);
        angleMotor.setInverted(angleInverted);
        angleMotor.setSensorPhase(angleSensorPhase);
        configMotionMagic(motionMagicConfigs);

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.selectProfileSlot(0, 0);

        encoder = new DutyCycleEncoder(encoderPort);

        SmartDashboard.putNumber(number.name() + "_kP", motionMagicConfigs[MotionMagicConfig.Kp.index]);
        SmartDashboard.putNumber(number.name() + "_kI", motionMagicConfigs[MotionMagicConfig.Ki.index]);
        SmartDashboard.putNumber(number.name() + "_kD", motionMagicConfigs[MotionMagicConfig.Kd.index]);
        SmartDashboard.putNumber(number.name() + "_kF", motionMagicConfigs[MotionMagicConfig.Kf.index]);
        SmartDashboard.putNumber(number.name() + "_sCurveStrength", motionMagicConfigs[MotionMagicConfig.SCurveStrength.index]);
        SmartDashboard.putNumber(number.name() + "_cruiseVelocity", motionMagicConfigs[MotionMagicConfig.CruiseVelocity.index]);
        SmartDashboard.putNumber(number.name() + "_maxAcceleration", motionMagicConfigs[MotionMagicConfig.MaxAcceleration.index]);
        SmartDashboard.putNumber(number.name() + "_allowableClosedLoopError", motionMagicConfigs[MotionMagicConfig.ClosedLoopError.index]);
        SmartDashboard.putNumber(number.name() + "_maxIntegralAccumulator", motionMagicConfigs[MotionMagicConfig.MaxIntegralAccumulator.index]);
        SmartDashboard.putNumber(number.name() + "_closedLoopPeakOutput", motionMagicConfigs[MotionMagicConfig.ClosedLoopPeakOutput.index]);
    }

    public void configMotionMagic(double[] motionMagicConfigs) {
        angleMotor.config_kP(0, motionMagicConfigs[MotionMagicConfig.Kp.index], TALON_TIMEOUT);
        angleMotor.config_kI(0, motionMagicConfigs[MotionMagicConfig.Ki.index], TALON_TIMEOUT);
        angleMotor.config_kD(0, motionMagicConfigs[MotionMagicConfig.Kd.index], TALON_TIMEOUT);
        angleMotor.config_kF(0, motionMagicConfigs[MotionMagicConfig.Kf.index], TALON_TIMEOUT);
        angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[MotionMagicConfig.SCurveStrength.index]);
        angleMotor.configMotionCruiseVelocity(motionMagicConfigs[MotionMagicConfig.CruiseVelocity.index]);
        angleMotor.configMotionAcceleration(motionMagicConfigs[MotionMagicConfig.MaxAcceleration.index]);
        angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[MotionMagicConfig.ClosedLoopError.index]);
        angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[MotionMagicConfig.MaxIntegralAccumulator.index]);
        angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[MotionMagicConfig.ClosedLoopPeakOutput.index]);
    }

    /**
     * Sets the module to the desired state.
     *
     * @param speed the desired speed of the module. [-1, 1]
     * @param angle the desired angle of the module.
     */
    public void set(double speed, Rotation2d angle) {
        loggerInputs.dSetpoint = speed * MAX_VELOCITY_METERS_PER_SECOND;

        SwerveModuleState optimized = SwerveModuleState.optimize(new SwerveModuleState(speed, angle), loggerInputs.aAngle);
        speed = optimized.speedMetersPerSecond;
        angle = optimized.angle;
        loggerInputs.aSetpoint = angle; // Setpoint angle of the wheel
        Rotation2d error = loggerInputs.aSetpoint.minus(loggerInputs.aAngle);

        driveMotor.set(ControlMode.PercentOutput, speed);

        angleMotor.set(ControlMode.MotionMagic, loggerInputs.aPosition + toFalconTicks(error));
    }

    /**
     * Converts the motor position to the angle of the module in the same coordinate system.
     *
     * @param ticks the position of the motor. [ticks]
     * @return the angle of the module.
     */
    public Rotation2d toWheelAbsoluteAngle(double ticks) {
        return new Rotation2d(((ticks / TICKS_PER_ROTATION) * 2 * Math.PI * ANGLE_GEAR_RATIO) % (2 * Math.PI));
    }

    /**
     * Converts the angle of the module to the motor position in the same coordinate system.
     *
     * @param angle the angle of the module.
     * @return the position of the motor. [ticks]
     */
    public int toFalconTicks(Rotation2d angle) {
        return (int) (((angle.getDegrees() / 360) * TICKS_PER_ROTATION) / ANGLE_GEAR_RATIO);
    }

    /**
     * Converts the absolute encoder position to the position
     * of the falcon in the same coordinate system.
     *
     * @param encoder the position of the encoder. [cycles]
     * @return the position of the falcon. [ticks]
     */
    public int absoluteEncoderToAbsoluteFalcon(double encoder) {
        return (int) (encoder * TICKS_PER_ROTATION / ANGLE_GEAR_RATIO);
    }

    /**
     * Gets the angle of the module.
     *
     * @return the angle of the module.
     */
    public Rotation2d getAngle() {
        return loggerInputs.aAngle;
    }

    /**
     * Gets the absolute encoder position.
     *
     * @return the absolute encoder position. [ticks]
     */
    public int getEncoderTicks() {
        return toFalconTicks(loggerInputs.encoderAngle);
    }

    /**
     * Gets the state of the module.
     *
     * @return the state of the module, comprised of speed [m/s] and angle.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(loggerInputs.dSetpoint, getAngle());
    }

    /**
     * Stops the module entirely.
     */
    public void stop() {
        angleMotor.neutralOutput();
        driveMotor.neutralOutput();
    }

    /**
     * Vroom vroom.
     * Used to break out modules.
     */
    public void vroom() {
        driveMotor.set(ControlMode.PercentOutput, 1);
        angleMotor.set(ControlMode.PercentOutput, 0.2);
    }

    /**
     * Gets the position of the module.
     * This is the relative position measured by the falcon. It's alright
     * to use this position because it is always differentiated.
     *
     * @return the position of the module.
     * Comprised of the distance [m], and the angle.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(loggerInputs.moduleDistance, getAngle());
    }

    @Override
    public void updateInputs() {
        loggerInputs.aPosition = angleMotor.getSelectedSensorPosition();
        loggerInputs.aAngle = toWheelAbsoluteAngle(loggerInputs.aPosition); // Angle of the wheel
        loggerInputs.encoderAngle = toWheelAbsoluteAngle(absoluteEncoderToAbsoluteFalcon(encoder.getAbsolutePosition()));
        loggerInputs.offsetAngle = toWheelAbsoluteAngle(offset);
        loggerInputs.aCurrent = angleMotor.getSupplyCurrent();

        loggerInputs.dVelocity = ((driveMotor.getSelectedSensorVelocity() / TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER)) * 10;
        loggerInputs.dCurrent = driveMotor.getSupplyCurrent();
        loggerInputs.moduleDistance = (driveMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION) * WHEEL_DIAMETER * Math.PI * DRIVE_REDUCTION;
    }

    @Override
    public void periodic() {
        if (!initializedOffset && encoder.isConnected()) {
            double newPosition = absoluteEncoderToAbsoluteFalcon(encoder.getAbsolutePosition()) - offset;
            angleMotor.setSelectedSensorPosition(newPosition);
            initializedOffset = true;
        }
        if (SmartDashboard.getBoolean("Swerve Tune Motion Magic", false)) {
            angleMotor.updatePIDF(0,
                    SmartDashboard.getNumber(number.name() + "_kP", motionMagicConfigs[MotionMagicConfig.Kp.index]),
                    SmartDashboard.getNumber(number.name() + "_kI", motionMagicConfigs[MotionMagicConfig.Ki.index]),
                    SmartDashboard.getNumber(number.name() + "_kD", motionMagicConfigs[MotionMagicConfig.Kd.index]),
                    SmartDashboard.getNumber(number.name() + "_kF", motionMagicConfigs[MotionMagicConfig.Kf.index]));

            angleMotor.configMotionCruiseVelocity(
                    SmartDashboard.getNumber(number.name() + "_cruiseVelocity",
                            motionMagicConfigs[MotionMagicConfig.CruiseVelocity.index]));
            angleMotor.configMotionAcceleration(
                    SmartDashboard.getNumber(number.name() + "_maxAcceleration",
                            motionMagicConfigs[MotionMagicConfig.MaxAcceleration.index]));
        }
    }

    @Override
    public String getSubsystemName() {
        return "SwerveModule_" + number;
    }

    public enum MotionMagicConfig {
        Kp(0),
        Ki(1),
        Kd(2),
        Kf(3),
        SCurveStrength(4),
        CruiseVelocity(5),
        MaxAcceleration(6),
        ClosedLoopError(7),
        MaxIntegralAccumulator(8),
        ClosedLoopPeakOutput(9);

        public final int index;

        MotionMagicConfig(int index) {
            this.index = index;
        }
    }
}
