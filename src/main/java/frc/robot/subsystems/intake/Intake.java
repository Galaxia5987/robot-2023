package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    private static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(Ports.Intake.ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMaxPIDController pidController = angleMotor.getPIDController();
    private final RelativeEncoder encoder = angleMotor.getEncoder();

    private Intake() {
        super(new IntakeLoggedInputs());

        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        motor.setInverted(Ports.Intake.POWER_INVERTED);

        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        angleMotor.setInverted(Ports.Intake.ANGLE_INVERTED);

        pidController.setP(IntakeConstants.kP);
        pidController.setI(IntakeConstants.kI);
        pidController.setD(IntakeConstants.kD);
        pidController.setOutputRange(-1, 1);
        pidController.setFeedbackDevice(encoder);
        pidController.setSmartMotionMaxAccel(IntakeConstants.INTAKE_ANGLE_MAX_ACCELERATION, 0);
        pidController.setSmartMotionMaxVelocity(IntakeConstants.INTAKE_ANGLE_VELOCITY, 0);

        motor.burnFlash();
        angleMotor.burnFlash();
    }

    /**
     * @return the INSTANCE of the Intake.
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    /**
     * @return the relative output.
     * Return the power that the motor applies. [%]
     */
    private double getPower() {
        return motor.get();
    }

    private double getAngleMotorVelocity() {
        return angleMotor.get();
    }

    /**
     * Set the motors' relative output.
     *
     * @param power is the power that the motor applies. [%]
     */
    public void setPower(double power) {
        motor.set(power);
    }

    public void setAnglePower(double power) {
        angleMotor.set(power);
    }

    /**
     * @return the motor's position. [degrees]
     */

    public double getAngle() {
        return encoder.getPosition() / IntakeConstants.ROTATIONS_PER_DEGREE;
    }

    /**
     * Sets the angles position.
     *
     * @param angle is the angle of the retractor. [degrees]
     */
    public void setAngle(double angle) {
        pidController.setReference(IntakeConstants.ROTATIONS_PER_DEGREE * angle, CANSparkMax.ControlType.kSmartMotion);
    }

    public void resetEncoder(){
        encoder.setPosition(0.25 * IntakeConstants.GEAR_RATIO);
    }

    public double getCurrent() {
        return loggerInputs.current;
    }

    /**
     * Update the logger inputs' value.
     */
    @Override
    public void updateInputs() {
        loggerInputs.power = getPower();
        loggerInputs.angle = getAngle();
        loggerInputs.velocity = getAngleMotorVelocity() * 5676;
        loggerInputs.current = angleMotor.getOutputCurrent();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
