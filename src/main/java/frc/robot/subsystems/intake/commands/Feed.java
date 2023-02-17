package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.BeamBreaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class Feed extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private final BeamBreaker beamBreaker = BeamBreaker.getInstance();
    private final double power;

    public Feed(double power) {
        this.power = power;
        addRequirements(intake);
    }

    /**
     * start the motor of the Intake
     */
    @Override
    public void execute() {
        intake.setPower(power);
        intake.setAnglePower(-0.05);
    }


    @Override
    public boolean isFinished() {
        return beamBreaker.isBeamBlocked();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
}
