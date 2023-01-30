package frc.robot.subsystems.intake;

import com.revrobotics.*;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    private static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(Ports.Intake.ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMaxPIDController pidController = angleMotor.getPIDController();
    private final RelativeEncoder encoder = angleMotor.getEncoder();
    private final UnitModel unitModel = new UnitModel(ConstantsIntake.TICKS_PER_DEGREE);

    private Intake() {
        super(new IntakeLoggedInputs());

        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        pidController.setP(ConstantsIntake.kP);
        pidController.setI(ConstantsIntake.kI);
        pidController.setD(ConstantsIntake.kD);
        motor.burnFlash();

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
     * Set the motors' relative output.
     * @param power is the power that the motor applies [%].
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * @return the relative output.
     * Return the power that the motor applies [%].
     */
    public double getPower() {
        return motor.get();
    }

    /**
     * Sets the angles position.
     * @param angle is the angle of the retractor [degrees].
     */
    public void setAngle(double angle) {
        pidController.setReference(unitModel.toTicks(angle), CANSparkMax.ControlType.kPosition);
    }

    /**
     * @return the motor's position [degrees].
     */
    //TODO: something doesn't look right here....

    public double getAngle() {
        return unitModel.toUnits(encoder.getPosition());
    }
    /**
     * Update the logger inputs' value.
     */
    @Override
    public void updateInputs() {
        loggerInputs.power = getPower();
        loggerInputs.angle = getAngle();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
