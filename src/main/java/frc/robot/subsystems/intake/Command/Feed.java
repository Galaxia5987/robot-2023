package frc.robot.subsystems.intake.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class Feed extends CommandBase {
    private final Intake intake;
    private final double power;
    private final double angle;

    public Feed(double power, double angle) {
        this.intake = Intake.getInstance();
        this.power = power;
        this.angle = angle;
        addRequirements(intake);
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
