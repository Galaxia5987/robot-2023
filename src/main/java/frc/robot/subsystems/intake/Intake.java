package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    public static Intake INSTANCE;
    private final CANSparkMax higherSparkMax = new CANSparkMax(Ports.Intake.LOWER_SPARK_MAX_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax lowerSparkMax = new CANSparkMax(Ports.Intake.HIGHER_SPARK_MAX_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID);

    private Intake() {
        super(new IntakeLoggedInputs());
        higherSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
        lowerSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /**
     * @return the INSTANCE of the Intake
     */
    public static Intake getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    /**
     * set the motors' relative output
     */
    public void setSparkMax(double power) {
        higherSparkMax.set(power);
        lowerSparkMax.set(power);
    }

    /**
     * @return the relative output of the higher motor.
     */
    public double getHigher() {
        return higherSparkMax.get();
    }

    /**
     * @return the relative output of the lower motor.
     */
    public double getLower() {
        return lowerSparkMax.get();
    }

    /**
     * set the wanted state for the retractor's solenoid.
     */
    public void setSolenoid(boolean state) {
        solenoid.set(state);
    }

    /**
     * toggle the Solenoid
     */
    public void toggleSolenoid() {
        solenoid.toggle();
    }

    /**
     * @return the Solenoid's current state
     */
    public boolean getSolenoid() {
        return solenoid.get();
    }

    /**
     * update the logger inputs' value
     */
    @Override
    public void updateInputs() {
        loggerInputs.solenoid = getSolenoid();
        loggerInputs.infMAX = getLower();
        loggerInputs.supMAX = getHigher();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
