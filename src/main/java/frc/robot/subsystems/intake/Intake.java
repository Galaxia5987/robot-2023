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
    private final CANSparkMax superiorSparkMax = new CANSparkMax(Ports.Intake.SUPERIOR_SPARK_MAX_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax inferiorSparkMax = new CANSparkMax(Ports.Intake.INFERIOR_SPARK_MAX_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID);

    private Intake(){
        super(new IntakeLoggedInputs());
        superiorSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
        inferiorSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setSparkMax(double power) {
        superiorSparkMax.set(power);
        inferiorSparkMax.set(power);
    }

    public double getSuperior() {
        return superiorSparkMax.get();
    }

    public double getInferior() {
        return inferiorSparkMax.get();
    }
    public void toggleSolenoid(){
        solenoid.toggle();
    }
    public boolean getSolenoid(){
        return solenoid.get();
    }

    @Override
    public void updateInputs() {
        IntakeLoggedInputs.solenoid= getSolenoid();
        IntakeLoggedInputs.infMAX= getInferior();
        IntakeLoggedInputs.supMAX= getSuperior();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
