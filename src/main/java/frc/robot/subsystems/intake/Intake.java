package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;


public class Intake extends LoggedSubsystem<IntakeLoggedInputs> {
    public static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID);
    private final DigitalInput beamBreaker = new DigitalInput(Ports.Intake.BEAMBREAKER);


    private Intake() {
        super(new IntakeLoggedInputs());
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        //TODO: add flush method
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
        solenoid.set(state);
    }

    public void closeRetractor(boolean state) {
        solenoid.set(state);
    }

    /**
     * toggle the Solenoid
     */
    public void toggleRetractor() {
        solenoid.toggle();
    }

    /**
     * @return the Solenoid's current state
     */
    public boolean getSolenoidState() {
        return solenoid.get();
    }

    public boolean getCurrentBeam() {
        return beamBreaker.get();
    }

    /**
     * this function is supposed to return whether a cube has passed through the intake( I have no idea if that's correct or not so please don't publicly execute me :) )
     */
    public String hasPassed() throws InterruptedException {
        if (getCurrentBeam() == false) {
            wait(3000);
            if (getCurrentBeam() == true) {
                return "a cube has passed";
            } else {
                return "a cube hasn't passed";
            }
        } else {
            return "a cube isn't passing";
        }
    }

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
