package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;


public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    public static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.LEFT_SOLENOID);
    private final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.RIGHT_SOLENOID);


    private Intake() {
        super(new IntakeLoggedInputs());
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.burnFlash();
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
     * @return the relative output of the lower motor.
     */

    /**
     * set the wanted state for the retractor's solenoid.
     */
    public void openRetractor(boolean state) {
        leftSolenoid.set(state);
    }

    public void closeRetractor(boolean state) {
        leftSolenoid.set(state);
    }

    /**
     * toggle the Solenoid
     */
    public void toggleRetractor() {
        leftSolenoid.toggle();
    }

    /**
     * @return the Solenoid's current state
     */
    public boolean getSolenoidState() {
        return leftSolenoid.get();
    }


    /**
     * this function is supposed to return whether a cube has passed through the intake( I have no idea if that's correct or not so please don't publicly execute me :) )
     */

    /**
     * update the logger inputs' value
     */
    @Override
    public void updateInputs() {
        loggerInputs.solenoidState = getSolenoidState();
        loggerInputs.motorPower = getPower();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
