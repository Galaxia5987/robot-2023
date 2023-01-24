package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    public static Intake INSTANCE;
    private final CANSparkMax superiorSparkMax = new CANSparkMax(Ports.Intake.SUPERIOR_SPARK_MAX_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax inferiorSparkMax = new CANSparkMax(Ports.Intake.INFERIOR_SPARK_MAX_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID);

    private Intake() {
        super(new IntakeLoggedInputs());
        superiorSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
        inferiorSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
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
        superiorSparkMax.set(power);
        inferiorSparkMax.set(power);
    }

    /**
     * @return the relative output of the higher motor.
     */
    public double getSuperior() {
        return superiorSparkMax.get();
    }

    /**
     * @return the relative output of the lower motor.
     */
    public double getInferior() {
        return inferiorSparkMax.get();
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
        IntakeLoggedInputs.solenoid = getSolenoid();
        IntakeLoggedInputs.infMAX = getInferior();
        IntakeLoggedInputs.supMAX = getSuperior();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
