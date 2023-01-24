package frc.robot.subsystems.intake.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.intake.Intake;

public class feed extends CommandBase {
    private Intake intake = Intake.getINSTANCE();

    public feed(Intake intake){
        addRequirements(intake);
    }

    /**
     * open the retractor for cubes
     */
    @Override
    public void initialize() {
        intake.setSolenoid(true);
    }

    /**
     * start the motor of the Intake
     */
    @Override
    public void execute() {
        intake.setSparkMax(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSparkMax(0);
    }
}
