package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.subsystems.intake.commands.HoldIntakeInPlace;
import frc.robot.utils.units.UnitModel;

public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    private static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(Ports.Intake.ANGLE_MOTOR);
    private final UnitModel unitModel = new UnitModel(IntakeConstants.TICKS_PER_DEGREE);

    private Command lastCommand = null;
    private boolean switchedToDefaultCommand = false;

    private Intake() {
        super(new IntakeLoggedInputs());
        angleMotor.configFactoryDefault();
        motor.restoreFactoryDefaults();

        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        motor.setInverted(Ports.Intake.POWER_INVERTED);
        for (int i = 1; i <= 6; i++) {
            motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.fromId(i), 500);
        }
        motor.burnFlash();

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        angleMotor.setInverted(Ports.Intake.ANGLE_INVERTED);
        angleMotor.config_kP(0, IntakeConstants.kP);
        angleMotor.config_kI(0, IntakeConstants.kI);
        angleMotor.config_kD(0, IntakeConstants.kD);
        angleMotor.config_kF(0, IntakeConstants.kF);
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
        loggerInputs.setpointPower = power;
    }

    private double getAngleMotorVelocity() {
        return unitModel.toVelocity(angleMotor.getSelectedSensorVelocity());
    }

    public void setAnglePower(double power) {
        angleMotor.set(TalonFXControlMode.PercentOutput, power);
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

    public void resetEncoder() {
        angleMotor.setSelectedSensorPosition(0);
    }

    public double getCurrent() {
        return loggerInputs.current;
    }

    public Command run(double power) {
        return new RunCommand(() -> this.setPower(power));
    }

    @Override
    public void periodic() {
        var currentCommand = getCurrentCommand();
        switchedToDefaultCommand = (currentCommand instanceof HoldIntakeInPlace) &&
                !(lastCommand instanceof HoldIntakeInPlace);
        lastCommand = currentCommand;
    }

    public boolean switchedToDefaultCommand() {
        return switchedToDefaultCommand;
    }

    /**
     * Update the logger inputs' value.
     */
    @Override
    public void updateInputs() {
        loggerInputs.power = getPower();
        loggerInputs.angle = getAngle();
        loggerInputs.velocity = getAngleMotorVelocity();
        loggerInputs.current = angleMotor.getSupplyCurrent();
        loggerInputs.anglePower = angleMotor.getMotorOutputPercent();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
