package frc.robot.subsystems.intake;

import com.revrobotics.*;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.units.UnitModel;


public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    private static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final UnitModel unitModel = new UnitModel(ConstantsIntake.TICKS_PER_RADIAN);
    private final CANSparkMax angleMotor = new CANSparkMax(Ports.Intake.ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkMaxPIDController pidController = angleMotor.getPIDController();


    private Intake() {
        super(new IntakeLoggedInputs());
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        pidController.setP(ConstantsIntake.kP);
        pidController.setI(ConstantsIntake.kI);
        pidController.setD(ConstantsIntake.kD);
        motor.burnFlash();
    }

    /**
     * @return the INSTANCE of the Intake
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    /**
     * set the motors' relative output
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * @return the relative output of the higher motor.
     */
    public double getPower() {
        return motor.get();
    }

    /**
     * sets the angles position
     */
    public void setAngle(double angle) {
        pidController.setReference(unitModel.toTicks(Math.toRadians(angle)), CANSparkMax.ControlType.kPosition);
    }

    /**
     * @return the motor's position
     */
    public double getAngle() {
        return Math.toDegrees(unitModel.toUnits(encoder.getPosition()));
    }


    /**
     * update the logger inputs' value
     */
    @Override
    public void updateInputs() {
        loggerInputs.motorPower = getPower();
        loggerInputs.angleMotorAngle = getAngle();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
