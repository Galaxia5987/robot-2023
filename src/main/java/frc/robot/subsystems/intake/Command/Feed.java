package frc.robot.subsystems.intake.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class Feed extends CommandBase {
    private Intake intake = Intake.getINSTANCE();
    private double power;

    public Feed(Intake intake, double power) {
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
        intake.setSparkMax(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSparkMax(0);
    }
}
