package frc.robot.subsystems.intake.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class Feed extends CommandBase {
    private Intake intake = Intake.getInstance();
    private double power;
    private double angle;


    public Feed(Intake intake, double power) {
        addRequirements(intake);
        this.power = power;
    }

    /**
     * open the retractor for cubes
     */
    @Override
    public void initialize() {
        intake.setAngle(angle);
    }

    /**
     * start the motor of the Intake
     */
    @Override
    public void execute() {
        intake.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setAngle(angle);
        intake.setPower(0);
    }
}
