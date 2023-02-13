package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;

public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    private static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(Ports.Intake.ANGLE_MOTOR);
    //private final CANSparkMax angleMotor = new CANSparkMax(Ports.Intake.ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    //private final SparkMaxPIDController pidController = angleMotor.getPIDController();
    //private final RelativeEncoder encoder = angleMotor.getEncoder();
    private final UnitModel unitModel = new UnitModel(IntakeConstants.TICKS_PER_DEGREE);

    private Intake() {
        super(new IntakeLoggedInputs());
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        motor.setInverted(Ports.Intake.POWER_INVERTED);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.setInverted(Ports.Intake.ANGLE_INVERTED);
        angleMotor.config_kP(0, IntakeConstants.kP);
        angleMotor.config_kI(0, IntakeConstants.kI);
        angleMotor.config_kD(0, IntakeConstants.kD);
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
     * @return the relative output.
     * Return the power that the motor applies. [%]
     */
    private double getPower() {
        return motor.get();
    }

    /**
     * Set the motors' relative output.
     *
     * @param power is the power that the motor applies. [%]
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * @return the motor's position. [degrees]
     */

    public double getAngle() {
        return unitModel.toUnits(angleMotor.getSelectedSensorPosition());
    }

    /**
     * Sets the angles position.
     *
     * @param angle is the angle of the retractor. [degrees]
     */
    public void setAngle(double angle) {
        angleMotor.set(ControlMode.Position, unitModel.toTicks(angle));
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
